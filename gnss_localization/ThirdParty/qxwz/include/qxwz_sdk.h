/*
 * Copyright (c) 2015-2020 QXSI. All rights reserved.
 */

#ifndef QXWZ_SDK_H__
#define QXWZ_SDK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "qxwz_types.h"

#if !defined(__WINDOWS__) && (defined(WIN32) || defined(WIN64) || defined(_MSC_VER) || defined(_WIN32))
#define __WINDOWS__
#endif

#ifdef __WINDOWS__

#define _STDCALL __stdcall
#if defined(QXWZ_EXPORT_SYMBOLS)
//#define QXWZ_PUBLIC(type)    __declspec(dllexport) type _STDCALL
#define QXWZ_PUBLIC(type) type
#else
//#define QXWZ_PUBLIC(type)   __declspec(dllimport) type _STDCALL
#define QXWZ_PUBLIC(type) type
#endif
#else /* !__WINDOWS__ */
#if defined(__GNUC__)
#define QXWZ_PUBLIC(type) __attribute__((visibility("default"))) type
#define ATTRIBUTE_WEAK __attribute__((weak))
#else
#define QXWZ_PUBLIC(type) type
#endif
#endif
#ifndef ATTRIBUTE_WEAK
#define ATTRIBUTE_WEAK
#endif

/*******************************************************************************
 * SDK status Definitions
 *******************************************************************************/

/**********************************
 * status code, 1 - 999
 **********************************/

/*
 * interface related, 1 - 100
 */
#define QXWZ_SDK_STAT_OK                                    0

/*
 * cap related, 101 - 200
 */
#define QXWZ_SDK_STAT_CAP_START_SUCC                        101
#define QXWZ_SDK_STAT_CAP_ACT_SUCC                          102
#define QXWZ_SDK_STAT_CAP_RESTART_SUCC                      103

/*
 * auth related, 201 - 300
 */
#define QXWZ_SDK_STAT_AUTH_SUCC                             201
#define QXWZ_SDK_STAT_LOCAL_AUTH_SUCC                       202

#define QXWZ_SDK_STAT_UPLOAD_FILE_COMP                      301 /* complete file uploading */
#define QXWZ_SDK_STAT_UPLOAD_FILE_IN_PROGRESS               302 /* file is uploading */


/**
 * define for enc conf callback
*/
#define QXWZ_SDK_STAT_ENC_CONF_ACCOUNT_CMD                  310 /* account request command */
#define QXWZ_SDK_STAT_ENC_CONF_ACCOUNT                      311 /* account info */
#define QXWZ_SDK_STAT_ENC_CONF_CAP_CMD                      312 /* cap request command */
#define QXWZ_SDK_STAT_ENC_CONF_LOCAL_CAP                    313 /* local cap info */
#define QXWZ_SDK_STAT_ENC_CONF_ONLINE_CAP                   314 /* online cap info */
#define QXWZ_SDK_STAT_ENC_CONF_NOTIFY                       315 /* status notify */

#define QXWZ_SDK_STAT_ENC_CONF_SET_ACCOUNT_OK               316 /* account has been set, go authenticating */
#define QXWZ_SDK_STAT_ENC_CONF_SET_CAP_OK                   317 /* cap has been set, go starting */
#define QXWZ_SDK_STAT_ENC_CONF_SET_NOTIFY                   318 /* remote status */

/*
 * biz custom related, 901 - 990
 */

#define QXWZ_SDK_STAT_BDS3_FMT_SUCC                         902


/*
 * unknown status code
 */
#define QXWZ_SDK_STAT_UNKNOWN                               999


/**********************************
 * error code, from -1 to -999
 **********************************/

/*
 * interface related, -1 - -100
 */
#define QXWZ_SDK_ERR_FAIL                                   -1 
#define QXWZ_SDK_ERR_INVALID_PARAM                          -2
#define QXWZ_SDK_ERR_INVALID_CONFIG                         -3
#define QXWZ_SDK_ERR_NOT_INITED                             -4
#define QXWZ_SDK_ERR_NOT_AUTHED                             -5
#define QXWZ_SDK_ERR_NOT_STARTED                            -6
#define QXWZ_SDK_ERR_AUTHING                                -7 
#define QXWZ_SDK_ERR_STARTING                               -8
#define QXWZ_SDK_ERR_ALREADY_INITED                         -9
#define QXWZ_SDK_ERR_ALREADY_AUTHED                         -10
#define QXWZ_SDK_ERR_ALREADY_STARTED                        -11
#define QXWZ_SDK_ERR_GETTING_COORD_SYS                      -12
#define QXWZ_SDK_ERR_SETTING_COORD_SYS                      -13
#define QXWZ_SDK_ERR_QUERYING_EXEC_STRATEGY                 -14
#define QXWZ_SDK_ERR_INVALID_SERV_CONF                      -15
#define QXWZ_SDK_ERR_INVALID_OSS_CONF                       -16
#define QXWZ_SDK_ERR_INVALID_NOSR_DATA_FMT_CONF             -17
#define QXWZ_SDK_ERR_INVALID_BROADCAST_INTERVAL             -18
#define QXWZ_SDK_ERR_ACTIVATING                             -19     /* sdk activate in progress */
#define QXWZ_SDK_ERR_RESUMING_DSK                           -20
#define QXWZ_SDK_ERR_IN_PROGRESS                            -21     /* common error code for something is still in progress */
#define QXWZ_SDK_ERR_ONLY_AVAILABLE_IN_DSK_MODE             -22     /* current interface only availble in DSK mode, please init SDK with DSK */

/*
 * network related, -101 - -200
 */
#define QXWZ_SDK_ERR_NETWORK_UNAVAILABLE                    -101

/*
 * cap related, -201 - -300
 */
#define QXWZ_SDK_ERR_GGA_OUT_OF_SERVICE_AREA                -201    /* the uploaded GGA is out of service area */
#define QXWZ_SDK_ERR_INVALID_GGA                            -202    /* the uploaded GGA is invalid */
#define QXWZ_SDK_ERR_CAP_START_FAIL                         -203    /* start capability failed */
#define QXWZ_SDK_ERR_CAP_GET_CONF_FAIL                      -204    /* get capability configuration failed */
#define QXWZ_SDK_ERR_CAP_NOT_FOUND                          -205    /* unsupported capability */
#define QXWZ_SDK_ERR_CAP_NOT_IN_SERVICE                     -206    /* generally error code for the capability which is not in service */
#define QXWZ_SDK_ERR_CAP_MANUAL_ACT_REQUIRED                -207    /* unactived capability, manual activation is required */
#define QXWZ_SDK_ERR_CAP_ACT_ON_TERM_REQUIRED               -208    /* unactived capability, terminal activation is required */
#define QXWZ_SDK_ERR_CAP_ALREADY_ACTIVATED                  -209    /* capability is already active */
#define QXWZ_SDK_ERR_CAP_CANNOT_ACT_ON_TERM                 -210    /* terminal activation is not allowed */
#define QXWZ_SDK_ERR_CAP_SYSTEM_ERROR                       -211    /* capability system error */
#define QXWZ_SDK_ERR_CAP_NOT_INCLUDE                        QXWZ_SDK_ERR_CAP_NOT_FOUND
#define QXWZ_SDK_ERR_CAP_PAUSE                              -213    /* suspended capability */
#define QXWZ_SDK_ERR_CAP_ACT_FAIL                           -214    /* terminal activate failed */
#define QXWZ_SDK_ERR_GGA_OUT_OF_CONTROL_AREA                -215    /* the uploaded GGA is out of control area, **** */
#define QXWZ_SDK_ERR_CAP_INACTIVE                           -216    /* generally error code for unactived capability */
#define QXWZ_SDK_ERR_CAP_EXPIRED                            -217    /* expired capability */
#define QXWZ_SDK_ERR_CAP_DISABLED                           -218    /* disabled capability */
#define QXWZ_SDK_ERR_CAP_NEED_AUDIT_DSK                     -219    /* please do real name authentication firstly */

/*
 * auth related, -301 - -400
 */
#define QXWZ_SDK_ERR_AUTH_FAIL                              -301
#define QXWZ_SDK_ERR_NO_AVAIL_ACC                           -302
#define QXWZ_SDK_ERR_MANUAL_BIND_REQUIRED                   -303
#define QXWZ_SDK_ERR_ACC_BEING_PROCESSED                    -304
#define QXWZ_SDK_ERR_UNMATCH_DID_DSK                        -305
#define QXWZ_SDK_ERR_ACC_NOT_BIND                           -307
#define QXWZ_SDK_ERR_ACC_EXPIRED                            -308
#define QXWZ_SDK_ERR_ACC_NOT_ENOUGH                         -309
#define QXWZ_SDK_ERR_ACC_UNSUPPORT_OP                       -310
#define QXWZ_SDK_ERR_INVAL_KEY                              -311    /* common error code for invalid account */
#define QXWZ_SDK_ERR_DSK_NOT_EXIST                          QXWZ_SDK_ERR_INVAL_KEY  /* deprecated */
#define QXWZ_SDK_ERR_ACC_INACTIVE                           -312
#define QXWZ_SDK_ERR_ACC_DUPLICATED                         -313
#define QXWZ_SDK_ERR_LOCAL_AUTH_FAIL                        -314
#define QXWZ_SDK_ERR_INVAL_QID                              -315
#define QXWZ_SDK_ERR_INVAL_ACCESS_TOKEN                     -316
#define QXWZ_SDK_ERR_PARSE_KEY_FAIL                         -317

/*
 * openapi related, -401 - -500
 */
#define QXWZ_SDK_ERR_CALL_API_FAIL                          -401
#define QXWZ_SDK_ERR_INVAL_API_RESP                         -402

/*
 * system related, -501 - -600
 */
#define QXWZ_SDK_ERR_OUT_OF_MEMORY                          -501
#define QXWZ_SDK_ERR_OUT_OF_STORAGE                         -502
#define QXWZ_SDK_ERR_FILE_NOT_FOUND                         -503
#define QXWZ_SDK_ERR_FILE_NO_ACCESS                         -504
#define QXWZ_SDK_ERR_INTERNAL_ERROR                         -505
#define QXWZ_SDK_ERR_SERV_FAULT                             -506
#define QXWZ_SDK_ERR_NOSR_SERVICE_STOP                      -507
#define QXWZ_SDK_ERR_FILE_OPEN_FAIL                         -508
#define QXWZ_SDK_ERR_FILE_SEEK_FAIL                         -509


/*
 * ssr decoder related, -601 - -700
 */
#define QXWZ_SDK_ERR_DEC_INIT_FAIL                          -601
#define QXWZ_SDK_ERR_DEC_INVALID_DATA                       -602
#define QXWZ_SDK_ERR_DEC_DECODE_FAIL                        -603
#define QXWZ_SDK_ERR_DEC_CRC_CHECK_FAIL                     -604
#define QXWZ_SDK_ERR_DEC_DECRYPT_FAIL                       -605
#define QXWZ_SDK_ERR_DEC_XOR_CHECK_FAIL                     -606
#define QXWZ_SDK_ERR_DEC_INTERNAL_ERR                       -607
#define QXWZ_SDK_ERR_DEC_INVALID_KEY                        -608
#define QXWZ_SDK_ERR_DEC_GET_KEY_FAIL                       -609
#define QXWZ_SDK_ERR_DEC_UNSUPP_TYPE                        -610    /* unsupported data type, it may be unsubscribed, or unrecognized */
#define QXWZ_SDK_ERR_DEC_INCOMP_DATA                        -611    /* incomplete data, waiting for more data, that is not a fatal error */
#define QXWZ_SDK_ERR_DEC_UNKNOWN                            -700


/*
 * section charge related, -801 - -900
 */
#define QXWZ_SDK_ERR_NO_STRATEGY_FOUND                      -801
#define QXWZ_SDK_ERR_DSK_CHECK_FAIL                         -802

/*
 * biz custom related, -901 - -990
 */

#define QXWZ_SDK_ERR_NOSR_DATA_FMT_TIMEOUT                  -921
#define QXWZ_SDK_ERR_NOSR_DATA_FMT_NOT_SUPPORT              -922
#define QXWZ_SDK_ERR_NOSR_DATA_FMT_SET_FAIL                 -923

/*
 * unknown error code
 */
#define QXWZ_SDK_ERR_UNKNOWN                                -99999

/**
 * Define the SDK status or error code for user.
*/
typedef enum {
    QXWZ_SDK_CODE_OK                          = QXWZ_SDK_STAT_OK, /* OK */
    QXWZ_SDK_ERR_AUDIT_PASSED                = QXWZ_SDK_STAT_OK, /* audit passed */
    QXWZ_SDK_ERR_AUDIT_INVALID_PARAM         = -1000, /* invalid parameter */
    QXWZ_SDK_ERR_AUDIT_PERMISSION_ERR        = -1001, /* permission error, please contact your administrator for more information */
    QXWZ_SDK_ERR_AUDIT_INTERNAL_ERR          = -1002, /* service inernal error, please contact your administrator for more information */
    QXWZ_SDK_ERR_AUDIT_ID_AGE_ERR            = -1003, /* the age of the ID number is illegal, please correct and retry */
    QXWZ_SDK_ERR_AUDIT_SERVICE_UNAVAILABLE   = -1004, /* the audit service is unavailable, please contact your administrator for more information */
    QXWZ_SDK_ERR_AUDIT_ID_CENTER_UNAVAILABLE = -1005, /* the audit center is under maintenance, please contact your administrator for more information */
    QXWZ_SDK_ERR_AUDIT_AUDIT_FAILED          = -1006, /* audit failed */
    QXWZ_SDK_ERR_AUDIT_NAME_ID_UNMATCH       = -1007, /* the name is unmatch the ID number */
    QXWZ_SDK_ERR_AUDIT_ID_UNAVAILABLE        = -1008, /* there is no related ID info in service center */
    QXWZ_SDK_ERR_AUDIT_USER_LOG_OFF          = -1009, /* the user is log off */
    QXWZ_SDK_ERR_AUDIT_USER_NOT_EXIST        = -1010, /* the user is not exist */
    QXWZ_SDK_ERR_AUDIT_ID_ALEADY_EXISTED     = -1011, /* the ID has been registered, please change another */
    QXWZ_SDK_ERR_AUDIT_ID_FORMAT_ERROR       = -1012, /* the format of the ID error */
    QXWZ_SDK_ERR_AUDIT_NAME_FORMAT_ERROR     = -1013, /* the format of the name error */
    QXWZ_SDK_ERR_AUDIT_GETTING               = QXWZ_SDK_ERR_IN_PROGRESS, /* requesting, please wait callback */

    QXWZ_SDK_ERR_ENC_CONF_DECODE_FAIL                = -1100,
    QXWZ_SDK_ERR_ENC_CONF_ENCODE_FAIL                = -1101,
    QXWZ_SDK_ERR_ENC_CONF_CHECK_FAIL                 = -1102,
    QXWZ_SDK_ERR_ENC_CONF_ACCOUNT_NOT_SAME           = -1103, /*the account you received is not equal to the account you input*/
    QXWZ_SDK_ERR_ENC_CONF_INVALID_OFFLINE_INFO_TYPE  = -1104, /* invalid data type while local authenticating */
    QXWZ_SDK_ERR_ENC_CONF_INVALID_OFFLINE_INFO_CONTENT  = -1105, /* invalid data content while local authenticating */


    QXWZ_SDK_ERR_UPLOADX_SERVER_ERROR                = -1200,
}QXWZ_SDK_STATUS_CODE_E;


/*******************************************************************************
 * SDK common macros definition
 *******************************************************************************/
/*
 * capability identifier
 */
#define QXWZ_SDK_CAP_ID_NOSR                    (1)
#define QXWZ_SDK_CAP_ID_NSSR                    (1 << 1)
#define QXWZ_SDK_CAP_ID_LSSR                    (1 << 2)
#define QXWZ_SDK_CAP_ID_PDR                     (1 << 3)
#define QXWZ_SDK_CAP_ID_VDR                     (1 << 4)
#define QXWZ_SDK_CAP_ID_EPH                     (1 << 5)
#define QXWZ_SDK_CAP_ID_QXSUPL                  (1 << 6)
#define QXWZ_SDK_CAP_ID_SIDS                    (1 << 7)
#define QXWZ_SDK_CAP_ID_UPLOADX                 (1 << 8)
#define QXWZ_SDK_CAP_ID_BASE_STATION            (1 << 9)
#define QXWZ_SDK_CAP_ID_ROVER_STATION           (1 << 10)
#define QXWZ_SDK_CAP_ID_GNSS_STATION            (1 << 11)
#define QXWZ_SDK_CAP_ID_ESIM                    (1 << 12)

/*
 * capability state
 */
#define QXWZ_SDK_CAP_STATE_INSERVICE            0
#define QXWZ_SDK_CAP_STATE_INACTIVE             1
#define QXWZ_SDK_CAP_STATE_SUSPENDED            2
#define QXWZ_SDK_CAP_STATE_EXPIRED              3
#define QXWZ_SDK_CAP_STATE_DISABLED             9

/*
 * capability activation method
 */
#define QXWZ_SDK_CAP_ACT_METHOD_AUTO            0
#define QXWZ_SDK_CAP_ACT_METHOD_MANUAL          1
#define QXWZ_SDK_CAP_ACT_METHOD_TERMINAL        2

/*
 * limitations
 */

#define CONFIG_CAPINFO_ENABLE                   1
#define CONFIG_PRINT_STDOUT_ENABLE              0
#define CONFIG_AUDIT_ENABLE                     1
#define CONFIG_OPENAPI_ENABLE                   1

#define QXWZ_SDK_MAX_CAPS                       8

#define QXWZ_SDK_VERSION_LEN                    128
#define QXWZ_SDK_MAX_KEY_LEN                    128
#define QXWZ_SDK_MAX_SECRET_LEN                 128
#define QXWZ_SDK_MAX_DEV_ID_LEN                 128
#define QXWZ_SDK_MAX_DEV_TYPE_LEN               128
#define QXWZ_SDK_MAX_QID_LEN                    128
#define QXWZ_SDK_MAX_ACCESS_TOKEN_LEN           512
#define QXWZ_SDK_MAX_SSR_KEY_STR_LEN            512
#define QXWZ_SDK_MAX_EXEC_STRATEGY_NUM          8
#define QXWZ_SDK_MAX_COORD_FRAME_NUM            8
#define QXWZ_SDK_MAX_COORD_FRAME_NAME           32
#define QXWZ_SDK_MAX_HOST_LEN                   128
#define QXWZ_SDK_DNS_SERVER_LEN                 16
#define QXWZ_SDK_MAX_HARDWARE_CAPS              4

#define QXWZ_SDK_MAX_AUDIT_ID_LEN               (18+1)  /* reserve 1 byte for \0 */
#define QXWZ_SDK_MAX_AUDIT_NAME_LEN             (255+1) /* reserve 1 byte for \0 */

#define QXWZ_SDK_NOSR_UPLOAD_GGA                10001    /* NOSR restart capbility */

typedef enum {
    QXWZ_SDK_DATA_TYPE_NONE                 = 0,    /* type guard */
    /* raw nosr data */
    QXWZ_SDK_DATA_TYPE_RAW_NOSR             = 1,



    QXWZ_SDK_DATA_TYPE_SIDS_INT             = 301,



    QXWZ_SDK_DATA_TYPE_CEIL                 = 0x7fffffff,    /* type guard */
} qxwz_sdk_data_type_e;

typedef enum {
    QXWZ_SDK_KEY_TYPE_NONE                  = 0,    /* key type guard */
    QXWZ_SDK_KEY_TYPE_AK                    = 1,    /* appkey */
    QXWZ_SDK_KEY_TYPE_DSK                   = 2,    /* DSK */
    QXWZ_SDK_KEY_TYPE_EXTERNAL               = 5,    /* allow there is no dsk and dss while initializing */
    QXWZ_SDK_KEY_TYPE_CEIL                          /* key type guard */
} qwxz_sdk_key_type_e;

typedef struct {
    qxwz_uint32_t caps_num;
    struct {
        qxwz_uint32_t cap_id;
        qxwz_uint8_t state;
        qxwz_uint8_t act_method;
        qxwz_uint64_t expire_time;

        /*
         * `keys_num`:
         *      0: no keys in this capability;
         *      1: it has only one key;
         *      2: it has two keys, check them one by one;
         */
        qxwz_uint32_t keys_num;
        struct {
            struct {
                qxwz_uint32_t week_num; /* week number of gps time */
                qxwz_uint32_t week_sec; /* week second of gps time */
            } start_time, expire_time;
        } keys_valid_period[2];
    } caps[QXWZ_SDK_MAX_CAPS];
} qxwz_sdk_cap_info_t;

typedef struct {
    qxwz_int32_t coord_sys_count;
    struct {
        qxwz_int32_t index;                                 /* identifier of the coordinate frame */
        qxwz_int32_t port;
        qxwz_char_t name[QXWZ_SDK_MAX_COORD_FRAME_NAME];    /* description of the coordinate frame */
    } coord_sys[QXWZ_SDK_MAX_COORD_FRAME_NUM];
    qxwz_int32_t serv_config_status;
    qxwz_int32_t curr_coord_sys_index;
} qxwz_sdk_coord_sys_info_t;

/*
 * Response for the RTCM execution plan query
 */
typedef struct {
    qxwz_int32_t exec_strategy_count;
    struct {
        qxwz_int32_t exec_type;
        qxwz_uint64_t exec_time;
        struct {
            qxwz_uint16_t year;
            qxwz_uint8_t  month;
            qxwz_uint8_t  day;
            qxwz_uint16_t minute;
        } exec_period;
    } exec_strategies[QXWZ_SDK_MAX_EXEC_STRATEGY_NUM];
} qxwz_sdk_exec_strategy_t;

/*
 * persistent callbacks
 */
typedef qxwz_void_t (*qxwz_sdk_data_callback_t)(qxwz_sdk_data_type_e type, const qxwz_void_t *data, qxwz_uint32_t len);
typedef qxwz_void_t (*qxwz_sdk_status_callback_t)(qxwz_int32_t status_code);
typedef qxwz_void_t (*qxwz_sdk_auth_callback_t)(qxwz_int32_t status_code, qxwz_sdk_cap_info_t *cap_info);
typedef qxwz_void_t (*qxwz_sdk_start_callback_t)(qxwz_int32_t status_code, qxwz_uint32_t cap_id);
typedef qxwz_void_t (*qxwz_sdk_stop_callback_t)(qxwz_int32_t status_code, qxwz_uint32_t cap_id);


/*
 * oneshot callbacks
 */
typedef qxwz_void_t (*qxwz_sdk_get_coord_sys_callback_t)(qxwz_int32_t status_code, qxwz_sdk_coord_sys_info_t *coord_sys_info);
typedef qxwz_void_t (*qxwz_sdk_set_coord_sys_callback_t)(qxwz_int32_t status_code);

typedef qxwz_void_t (*qxwz_sdk_query_exec_strategy_callback_t)(qxwz_int32_t status_code, qxwz_sdk_exec_strategy_t *strategy);
typedef qxwz_void_t (*qxwz_sdk_resume_dsk_callback_t)(qxwz_int32_t status_code);

typedef struct {
    qxwz_int32_t caps_num;
    struct {
        qxwz_uint32_t cap_id;
        qxwz_int32_t result;    /*  QXWZ_SDK_STAT_CAP_ACT_SUCC       :activate cap successfully
                                    QXWZ_SDK_ERR_CAP_EXPIRED         :cap is expired
                                    QXWZ_SDK_ERR_CAP_NOT_INCLUDE     :there is no current cap in DSK
                                    QXWZ_SDK_ERR_CAP_SYSTEM_ERROR    :cap system error, please contact your administrator
                                    QXWZ_SDK_ERR_CAP_NEED_AUDIT_DSK  :please audit DSK firstly(real name authentication) by qxwz_sdk_audit_dsk
                                    QXWZ_SDK_ERR_CAP_ACT_FAIL        :activate cap fail, please contact your administrator
                                */
    } caps[QXWZ_SDK_MAX_CAPS];
} qxwz_sdk_activate_resp_t;

typedef qxwz_void_t (*qxwz_sdk_activate_callback_t)(qxwz_int32_t status_code, qxwz_sdk_activate_resp_t *act_resp);


typedef struct {
    qwxz_sdk_key_type_e key_type;                         /* AK/DSK */
    qxwz_char_t key[QXWZ_SDK_MAX_KEY_LEN];
    qxwz_char_t secret[QXWZ_SDK_MAX_SECRET_LEN];
    qxwz_char_t dev_id[QXWZ_SDK_MAX_DEV_ID_LEN];
    qxwz_char_t dev_type[QXWZ_SDK_MAX_DEV_TYPE_LEN];
    qxwz_sdk_data_callback_t data_cb;
    qxwz_sdk_status_callback_t status_cb;
    qxwz_sdk_auth_callback_t auth_cb;
    qxwz_sdk_start_callback_t start_cb;
} qxwz_sdk_config_t;

typedef enum {
    QXWZ_SDK_CONF_SERV = 0,
    QXWZ_SDK_CONF_OSS,
    QXWZ_SDK_CONF_NOSR_DATA_FORMAT,
    QXWZ_SDK_CONF_NET_TIMEOUT,           /* the value type should be `qxwz_uint32_t *`, 1 ~ 600s */
    QXWZ_SDK_CONF_DNS_SERVER,
    QXWZ_SDK_CONF_STARX_PUB_KEY
} qxwz_sdk_conf_t;

typedef struct {
    qxwz_char_t openapi_host[QXWZ_SDK_MAX_HOST_LEN];
    qxwz_uint32_t openapi_port;

    qxwz_char_t oss_host[QXWZ_SDK_MAX_HOST_LEN];
    qxwz_uint32_t oss_port;
} qxwz_sdk_serv_conf_t;

typedef struct {
    qxwz_uint32_t oss_heartbeat_interval;
    qxwz_uint32_t oss_reconnect_interval;
} qxwz_sdk_oss_conf_t;

typedef enum {
    QXWZ_SDK_NOSR_QX_FORMAT,
    QXWZ_SDK_NOSR_STD_FORMAT       /* default bds3 format */
} qxwz_sdk_nosr_data_format_t;

typedef struct {
    qxwz_char_t dns_server[QXWZ_SDK_DNS_SERVER_LEN];
} qxwz_sdk_dns_serv_conf_t;


/* sids data structure */
typedef struct {
    qxwz_uint8_t sig_id;               /* Signal number */
    qxwz_uint8_t sig_int;              /* Signal integrity */
} qxwz_sids_sig_info_t;

typedef struct {
    qxwz_uint8_t sat_id;               /* Satellite number */
    qxwz_uint8_t sig_num;              /* the total signal numbers of a satellite */
    qxwz_sids_sig_info_t *sig_info;
} qxwz_sids_sat_info_t;

typedef struct {
    qxwz_uint8_t ctrl_meta;            /* Control byte */
    qxwz_uint8_t multi_msg_flag;       /* MultipleMessage */
    qxwz_uint8_t gnss_sys;             /* 'G' = GPS, 'R' = GLONASS, 'C' = COMPASS(BDS), 'E' = GALILEO  'N' = NULL */
    struct {
        qxwz_uint16_t week_num;
        qxwz_uint32_t week_sec;        /* GPS seconds */
    } gps_time;
    qxwz_uint8_t sat_num;              /* Satellite number */
    qxwz_sids_sat_info_t *sat_info;
} qxwz_sids_sid_info_t;                /* Satellite integrity data */


typedef struct{
    qxwz_uint8_t name[QXWZ_SDK_MAX_AUDIT_NAME_LEN];
    qxwz_uint8_t id[QXWZ_SDK_MAX_AUDIT_ID_LEN];
}qxwz_sdk_audit_config_t;

/**
  * The callback for caller to receive audit result.
  *
  * @param[in] code: SDK return code transfered from service side;
  */
typedef qxwz_void_t (*qxwz_sdk_audit_dsk_cb_t)(qxwz_int32_t code);



/**
 * get SDK version string
 *
 * @return:
 *      pointer of the version string;
 */
QXWZ_PUBLIC(const qxwz_char_t*) qxwz_sdk_version();

/**
 * get SDK build information
 *
 * @return:
 *      pointer of the build information string;
 */
QXWZ_PUBLIC(const qxwz_char_t*) qxwz_sdk_get_build_info();

/**
 * set common configs
 *
 * @param[in]  type: type of the configuration, see definition of `qxwz_sdk_conf_t`;
 * @param[in]  conf: pointer to the specific parameter;
 * 
 * @return:
 *      0: succeeds;
 *     <0: fails;
 */
QXWZ_PUBLIC(qxwz_int32_t) qxwz_sdk_config(qxwz_sdk_conf_t type, qxwz_void_t* conf);

/**
 * initialize SDK
 *
 * @param[in]  config: collect of account and callbacks;
 *
 * @return:
 *      0: succeeds;
 *     <0: fails;
 *
 * Notice:
 *      If something wrong happens to the SDK, the error code would be published through
 *      the `status_cb` callback.
 */
QXWZ_PUBLIC(qxwz_int32_t) qxwz_sdk_init(qxwz_sdk_config_t *config);


/**
 * do authentication
 *
 * @return:
 *      0: succeeds;
 *     <0: fails;
 *
 * Notice:
 *      The result of authentication would be notified through the `auth_cb`,
 *      which is registered in the `qxwz_sdk_init` call.
 */
QXWZ_PUBLIC(qxwz_int32_t) qxwz_sdk_auth();

/**
 * start the service corresponding to the specific capability
 *
 * @param[in]  cap_id: identifier of the capability;
 *
 * @return:
 *      0: succeeds;
 *     <0: fails;
 *
 * Notice:
 *      The result of start a capability would be notified through the `start_cb`,
 *      which is registered in the `qxwz_sdk_init` call.
 */
QXWZ_PUBLIC(qxwz_int32_t) qxwz_sdk_start(qxwz_uint32_t cap_id);


/**
 * stop the service corresponding to the specific capability
 *
 * @param[in]  cap: identifier of the capability;
 *
 * @return:
 *      0: succeeds;
 *     <0: fails;
 */
QXWZ_PUBLIC(qxwz_int32_t) qxwz_sdk_stop(qxwz_uint32_t cap);

/**
 * cleanup SDK
 *
 * @return:
 *      0: succeeds;
 *     <0: fails;
 */
QXWZ_PUBLIC(qxwz_int32_t) qxwz_sdk_cleanup();

/**
 * upload gga
 *
 * @param[in]  gga: pointer of GGA string;
 * @param[in]  len: length of the string;
 *
 * @return:
 *      0: succeeds;
 *     <0: fails;
 */
QXWZ_PUBLIC(qxwz_int32_t) qxwz_sdk_upload_gga(const qxwz_char_t *gga, qxwz_uint32_t len);


/**
 * get detailed information of the capabilities bound to the account
 *
 * @param[in]  cap_info: structure pointer of capability information;
 *
 * @return:
 *      0: succeeds;
 *     <0: fails;
 */
QXWZ_PUBLIC(qxwz_int32_t) qxwz_sdk_get_cap_info(qxwz_sdk_cap_info_t *cap_info);

/**
 * get coordinate system information, including count of coordinate system, 
 * detail infomation of each coordinate system and index of current coordinate system in use.
 * this function is asynchronous, you will get the result in the callback function after it is fetched.
 *
 *  @param[in] get_coord_sys_cb: the callback which will be called after fetching information from server;
 *
 *  @return:
 *       0: succeeds;
 *      <0: fails;
 */
QXWZ_PUBLIC(qxwz_int32_t) qxwz_sdk_get_coord_sys(qxwz_sdk_get_coord_sys_callback_t get_coord_sys_cb);

/**
 *  set coordinate system by index.
 *  this function is asynchronous, you will get the setting result code in the callback function.
 *  if you set coordinate system after starting NOSR, you need restart NOSR after setting successfully.
 *
 *  @param[in] coord_sys_index: the index of coordinate system;
 *  @param[in] set_coord_sys_cb: the callback which will be called after setting is done;
 *
 *  @return:
 *       0: succeeds;
 *      <0: fails;
 */
QXWZ_PUBLIC(qxwz_int32_t) qxwz_sdk_set_coord_sys(qxwz_uint32_t coord_sys_index, qxwz_sdk_set_coord_sys_callback_t set_coord_sys_cb);

/**
 *  query RTCM execution plans of the current account.
 *  this function is asynchronous, you will get the result in the callback function.
 *
 *  @param[in] query_exec_strategy_cb: the callback which will be called after query is done;
 *
 *  @return:
 *       0: succeeds;
 *      <0: fails;
 */
QXWZ_PUBLIC(qxwz_int32_t) qxwz_sdk_query_exec_strategy(qxwz_sdk_query_exec_strategy_callback_t query_exec_strategy_cb);

/**
 *  resume the paused account.
 *  this function is asynchronous, you will get the result in the callback function.
 *
 *  @param[in] resume_dsk_cb: the callback which will be called after resuming is done;
 *
 *  @return:
 *       0: succeeds;
 *      <0: fails;
 */
QXWZ_PUBLIC(qxwz_int32_t) qxwz_sdk_resume_dsk(qxwz_sdk_resume_dsk_callback_t resume_dsk_cb);


/**
 *  activate the capabilities from terminal
 *
 *  @param[in] caps: the array of capabilities;
 *  @param[in] caps_num: the number of capbilities;
 *  @param[in] act_cb: the callback to notify the activation result;
 *
 *  @return:
 *       0: succeeds;
 *      <0: fails;
 */
QXWZ_PUBLIC(qxwz_int32_t) qxwz_sdk_activate(const qxwz_uint32_t *caps, qxwz_int32_t caps_num, qxwz_sdk_activate_callback_t act_cb);



/**
  * This is defined to do the real name authentication by DSK, so you should pass the qxwz_sdk_auth 
  * firstly, or else there is no DSK to do the real name authentication.
  * The audit result will feedback through the audit_dsk_cb, please refer to the definition of qxwz_openapi_audit_dsk_cb_t
  * for more result details.
  * 
  * @param[in] config: the name and id which will be audit by DSK.
  * @param[in] audit_dsk_cb: the callback to report audit result;
  *
  * @return:
         QXWZ_SDK_STAT_OK           :function success to excute, please wait the audit_dsk_cb for the details
         QXWZ_SDK_ERR_INVALID_PARAM :invalid parameters, please check the name, id and audit_dsk_cb
         QXWZ_SDK_ERR_AUDITING      :in auditing, please check the result of audit_dsk_cb
         QXWZ_SDK_ERR_NOT_AUTHED    :DSK has not authenticated yet, please do qxwz_sdk_auth firstly
         QXWZ_SDK_XXXX              :please refer to the SDK error code for more details.
  *         
  */
QXWZ_PUBLIC(qxwz_int32_t) qxwz_sdk_audit_dsk(const qxwz_sdk_audit_config_t *config, qxwz_sdk_audit_dsk_cb_t audit_dsk_cb);



#ifdef __cplusplus
}
#endif

#endif
