#include "rk_row_nav.h"
#include <math.h>
#include <string.h>
int main(void) {
    rk_row_nav_config_t config; rk_row_nav_config_init(&config);
    rk_row_nav_result_t result; rk_row_nav_result_init(&result);
    rk_row_nav_handle_t handle = 0;
    if (rk_row_nav_create(&config, &handle) != RK_ROW_NAV_INVALID_ARGUMENT || handle) return 1;
    if (!strlen(rk_row_nav_get_last_error()) || !isnan(result.lateral_error_m)) return 2;
    if (rk_row_nav_process(0, 0, &result) != RK_ROW_NAV_INVALID_ARGUMENT) return 3;
    if (result.observation_status != RK_ROW_NAV_OBSERVATION_INVALID) return 4;
    if (strcmp(rk_row_nav_version(), "0.2.0") || !rk_row_nav_now_ns()) return 5;
    rk_row_nav_attitude_t attitude; rk_row_nav_attitude_init(&attitude);
    if (attitude.valid || attitude.struct_size != sizeof(attitude)) return 6;
    if (rk_row_nav_dynamic_bev_enabled(0) != RK_ROW_NAV_INVALID_ARGUMENT) return 7;
    if (rk_row_nav_process_with_attitude(0,0,&attitude,&result) != RK_ROW_NAV_INVALID_ARGUMENT) return 8;
    rk_row_nav_destroy(0);
    return 0;
}
