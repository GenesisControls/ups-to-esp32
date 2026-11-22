/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "auth.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "mbedtls/sha256.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "netmgr_ap.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "auth";

/* Same namespace as the Wi-Fi credentials, so netmgr_factory_reset -- the
 * five-press BOOT gesture -- erases the login with them. */
#define NVS_NS      "upsa"
#define KEY_USER    "auth_user"
#define KEY_SALT    "auth_salt"
#define KEY_HASH    "auth_hash"
/* Set alongside the credentials when auth_init() seeded the compiled-in
 * default; cleared the moment anyone sets a login of their own. Kept in NVS
 * rather than in RAM so the notice survives a restart. */
#define KEY_DEFAULT "auth_dflt"

/* Empty unless the build sets them; see components/auth/Kconfig.projbuild.
 * Defined here too so this file still compiles if that Kconfig is absent. */
#ifndef CONFIG_UPSA_DEFAULT_ADMIN_USER
#define CONFIG_UPSA_DEFAULT_ADMIN_USER ""
#endif
#ifndef CONFIG_UPSA_DEFAULT_ADMIN_PASS
#define CONFIG_UPSA_DEFAULT_ADMIN_PASS ""
#endif

/* Salted and iterated, so a copy of the flash does not hand over a password
 * people may use elsewhere. The S3 hashes in hardware: 4096 rounds is a few
 * milliseconds per check, and a guesser pays it every time. */
#define HASH_ROUNDS 4096

static authc_t s_a;
static SemaphoreHandle_t s_lock;
static bool s_default;          /* the login is still the compiled-in default */

static void hash_pass(const uint8_t salt[AUTH_SALT_LEN], const char *pass,
                      uint8_t out[AUTH_HASH_LEN])
{
    mbedtls_sha256_context c;
    mbedtls_sha256_init(&c);
    mbedtls_sha256_starts(&c, 0);
    mbedtls_sha256_update(&c, salt, AUTH_SALT_LEN);
    mbedtls_sha256_update(&c, (const unsigned char *)pass, strlen(pass));
    mbedtls_sha256_finish(&c, out);
    for (int i = 0; i < HASH_ROUNDS; i++) {
        mbedtls_sha256_starts(&c, 0);
        mbedtls_sha256_update(&c, out, AUTH_HASH_LEN);
        mbedtls_sha256_update(&c, salt, AUTH_SALT_LEN);
        mbedtls_sha256_finish(&c, out);
    }
    mbedtls_sha256_free(&c);
}

/* Who is on the other end of a socket, as the lockout's key: the IPv4
 * address, or an IPv6 one's /64 prefix folded to 32 bits; 0 if the socket
 * cannot say. The prefix, not the address: a host can take any number of
 * addresses in its own /64, and each would start a fresh count. */
uint32_t auth_src_of_fd(int fd)
{
    struct sockaddr_storage ss;
    socklen_t sl = sizeof(ss);
    if (fd < 0 || getpeername(fd, (struct sockaddr *)&ss, &sl) != 0) return 0;
    uint32_t v4 = netmgr_sockaddr_v4(&ss, sl);
    if (v4) return v4;
#if LWIP_IPV6
    if (ss.ss_family == AF_INET6) {
        const uint32_t *w = (const uint32_t *)&((struct sockaddr_in6 *)&ss)->sin6_addr;
        uint32_t k = w[0] ^ w[1];
        return k ? k : 1;
    }
#endif
    return 0;
}

static int64_t now_us(void) { return esp_timer_get_time(); }

static esp_err_t set_creds(const char *user, const char *pass, bool is_default);

esp_err_t auth_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    memset(&s_a, 0, sizeof(s_a));

    /* The namespace is absent on a board that has never written to it. That
     * is not an error, and -- unlike before -- not a reason to return early:
     * the default login below has to be seeded on exactly that board. */
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        char user[AUTH_USER_MAX + 1] = "";
        uint8_t salt[AUTH_SALT_LEN], hash[AUTH_HASH_LEN];
        size_t ul = sizeof(user), sl = sizeof(salt), hl = sizeof(hash);
        if (nvs_get_str(h, KEY_USER, user, &ul) == ESP_OK &&
            nvs_get_blob(h, KEY_SALT, salt, &sl) == ESP_OK && sl == AUTH_SALT_LEN &&
            nvs_get_blob(h, KEY_HASH, hash, &hl) == ESP_OK && hl == AUTH_HASH_LEN && user[0]) {
            authc_set(&s_a, user, salt, hash);
            uint8_t d = 0;
            if (nvs_get_u8(h, KEY_DEFAULT, &d) == ESP_OK) s_default = d != 0;
        }
        nvs_close(h);
    }

    /* A build may ship a login so the board is usable the moment it powers up
     * and the setup page need not insist on one. ONLY when nothing is stored:
     * a login someone chose is never overwritten, and once they change the
     * default this stops firing because a login then exists. */
    if (!authc_enabled(&s_a) && CONFIG_UPSA_DEFAULT_ADMIN_USER[0]) {
        esp_err_t serr = set_creds(CONFIG_UPSA_DEFAULT_ADMIN_USER,
                                   CONFIG_UPSA_DEFAULT_ADMIN_PASS, true);
        if (serr == ESP_OK) {
            ESP_LOGW(TAG, "seeded the built-in default login (user '%s') -- "
                          "change the password on the web page",
                     CONFIG_UPSA_DEFAULT_ADMIN_USER);
        } else {
            ESP_LOGE(TAG, "could not seed the default login: %s", esp_err_to_name(serr));
        }
    }

    ESP_LOGI(TAG, "login %s%s", authc_enabled(&s_a) ? "ON: changes need a username and password"
                                                    : "off (optional; set one on the web page)",
             s_default ? " (still the built-in default)" : "");
    return ESP_OK;
}

bool auth_enabled(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool on = authc_enabled(&s_a);
    xSemaphoreGive(s_lock);
    return on;
}

static esp_err_t set_creds(const char *user, const char *pass, bool is_default)
{
    bool clear = !user || !user[0];
    if (!clear && !authc_shape_ok(user, pass)) return ESP_ERR_INVALID_ARG;

    uint8_t salt[AUTH_SALT_LEN] = {0}, hash[AUTH_HASH_LEN] = {0};
    if (!clear) {
        esp_fill_random(salt, sizeof(salt));
        hash_pass(salt, pass, hash);
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    if (clear) {
        nvs_erase_key(h, KEY_USER);
        nvs_erase_key(h, KEY_SALT);
        nvs_erase_key(h, KEY_HASH);
        nvs_erase_key(h, KEY_DEFAULT);
    } else {
        nvs_set_str(h, KEY_USER, user);
        nvs_set_blob(h, KEY_SALT, salt, sizeof(salt));
        nvs_set_blob(h, KEY_HASH, hash, sizeof(hash));
        nvs_set_u8(h, KEY_DEFAULT, is_default ? 1 : 0);
    }
    err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) return err;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    authc_set(&s_a, clear ? "" : user, salt, hash);     /* also logs everyone out */
    s_default = !clear && is_default;
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "login %s", clear ? "turned off" : "set");
    return ESP_OK;
}

/* Anything a person sets is by definition not the built-in default, so this
 * clears the flag. auth_init() is the only caller that ever sets it. */
esp_err_t auth_set_credentials(const char *user, const char *pass)
{
    return set_creds(user, pass, false);
}

bool auth_is_default(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool d = s_default && authc_enabled(&s_a);
    xSemaphoreGive(s_lock);
    return d;
}

static auth_result_t from_core(authc_result_t r)
{
    return r == AUTHC_OK ? AUTH_OK : r == AUTHC_LOCKED ? AUTH_LOCKED : AUTH_BAD;
}

bool auth_username(char *out, size_t cap)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool on = authc_enabled(&s_a);
    if (on) strlcpy(out, s_a.user, cap);
    xSemaphoreGive(s_lock);
    return on;
}

auth_result_t auth_check(uint32_t src, const char *user, const char *pass)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    authc_result_t r = authc_check(&s_a, src, user, pass, now_us(), hash_pass);
    xSemaphoreGive(s_lock);
    return from_core(r);
}

auth_result_t auth_login(uint32_t src, const char *user, const char *pass,
                         char token[AUTH_TOKEN_LEN + 1])
{
    uint8_t rnd[AUTH_TOKEN_LEN / 2];
    esp_fill_random(rnd, sizeof(rnd));
    for (size_t i = 0; i < sizeof(rnd); i++) sprintf(token + 2 * i, "%02x", rnd[i]);
    token[AUTH_TOKEN_LEN] = '\0';

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int64_t t = now_us();
    authc_result_t r = authc_check(&s_a, src, user, pass, t, hash_pass);
    if (r == AUTHC_OK) authc_issue(&s_a, token, t);
    xSemaphoreGive(s_lock);
    if (r != AUTHC_OK) token[0] = '\0';
    return from_core(r);
}

bool auth_token_ok(const char *token)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool ok = authc_token_ok(&s_a, token, now_us());
    xSemaphoreGive(s_lock);
    return ok;
}

void auth_logout(const char *token)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    authc_revoke(&s_a, token);
    xSemaphoreGive(s_lock);
}

authc_nut_t auth_nut_write(uint32_t src, const char *user, const char *pass)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    authc_nut_t r = authc_nut_write(&s_a, src, user, pass, now_us(), hash_pass);
    xSemaphoreGive(s_lock);
    return r;
}

bool auth_origin_ok(const char *origin, const char *host)
{
    return authc_origin_ok(origin, host);
}
