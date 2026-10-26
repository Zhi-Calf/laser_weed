/* Plain C consumer: no project-private headers and no OpenCV dependency. */
#include <rk_row_nav.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char** argv) {
    if (argc == 2 && strcmp(argv[1], "--version") == 0) {
        puts(rk_row_nav_version()); return 0;
    }
    if (argc != 8) {
        fprintf(stderr, "usage: %s onnx|rknn model intrinsics.yaml bev.yaml frame.bgr width height\n", argv[0]);
        return 1;
    }
    if (strcmp(argv[1], "onnx") && strcmp(argv[1], "rknn")) return 1;
    char *end_w, *end_h;
    long width = strtol(argv[6], &end_w, 10), height = strtol(argv[7], &end_h, 10);
    if (*end_w || *end_h || width <= 0 || height <= 0 || width > 32768 || height > 32768) return 1;
    size_t size = (size_t)width * (size_t)height * 3;
    FILE* fp = fopen(argv[5], "rb");
    if (!fp) { perror("frame"); return 1; }
    unsigned char* data = (unsigned char*)malloc(size);
    if (!data) { fclose(fp); return 1; }
    size_t read_count = fread(data, 1, size, fp);
    int extra = fgetc(fp); fclose(fp);
    if (read_count != size || extra != EOF) { fprintf(stderr, "frame must be tightly packed BGR888\n"); free(data); return 1; }
    rk_row_nav_config_t config;
    rk_row_nav_config_init(&config);
    config.backend = !strcmp(argv[1], "rknn") ? RK_ROW_NAV_RKNN : RK_ROW_NAV_ONNX;
    config.model_path = argv[2]; config.intrinsic_path = argv[3]; config.bev_path = argv[4];
    rk_row_nav_handle_t handle = NULL;
    int code = rk_row_nav_create(&config, &handle);
    if (code) { fprintf(stderr, "%s\n", rk_row_nav_get_last_error()); free(data); return 2; }
    rk_row_nav_frame_t frame; rk_row_nav_frame_init(&frame);
    frame.data = data; frame.data_size_bytes = size;
    frame.width = (uint32_t)width; frame.height = (uint32_t)height; frame.stride_bytes = (uint32_t)width * 3;
    /* File replay: timestamp at submission. Real capture uses capture time. */
    frame.capture_time_ns = rk_row_nav_now_ns(); frame.frame_id = 1;
    rk_row_nav_result_t result; rk_row_nav_result_init(&result);
    code = rk_row_nav_process(handle, &frame, &result);
    if (code) fprintf(stderr, "%s\n", rk_row_nav_get_last_error());
    else if (result.observation_status == RK_ROW_NAV_OBSERVATION_VALID)
        printf("lateral_m=%f heading_rad=%f confidence=%f\n", result.lateral_error_m, result.heading_error_rad, result.confidence);
    else printf("observation unavailable: status=%u\n", result.observation_status);
    printf("processing_ms=%.3f frame_age_ms=%.3f\n", result.processing_ms, result.frame_age_ms);
    rk_row_nav_destroy(handle); free(data);
    return code ? 3 : 0;
}
