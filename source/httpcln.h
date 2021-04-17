#ifndef HTTPCLN_H_
#define HTTPCLN_H_

#include "lwip/sys.h"

#include <stdint.h>

typedef uint8_t flag_t;

#define HTTPCLN_MAX_HTTP_HOST_LENGTH		64
#define HTTPCLN_MAX_HTTP_URL_PATH_LENGTH	256

#define HTTPCLN_TIMEOUT_SEC					15
#define HTTPCLN_STALL_TIMEOUT_SEC			30
#define HTTPCLN_TCP_WINDOW_SIZE				1460

typedef flag_t (*httpcln_processdata_cb)(uint8_t *buf, uint16_t buf_size, uint32_t content_length, uint32_t content_bytes_received, void *param);
typedef flag_t (*httpcln_is_cancelled_cb)(void);

#define HTTPCLN_HEADER_KEY_EMPTY			0
#define HTTPCLN_HEADER_KEY_AUTH_BEARER		1
typedef uint8_t httpcln_header_key_t;

typedef struct {
	httpcln_header_key_t key;
	char *value;
} httpcln_header_entry_t;

typedef struct {
	uint32_t max_resp_content_length;
	uint32_t content_length_for_progress;
	TickType_t time_header_received_at;
	uint32_t continue_dl_from_byte;

	httpcln_processdata_cb processdata_cb;
	void *processdata_cb_param;

	httpcln_is_cancelled_cb is_cancelled_cb;

	uint8_t custom_header_entries_count;
	httpcln_header_entry_t *custom_header_entries;
} httpcln_params_t;

#define HTTPCLN_RESULT_UNAUTHORIZED			-3
#define HTTPCLN_RESULT_UNALLOWED			-2
#define HTTPCLN_RESULT_CANCELLED			-1
#define HTTPCLN_RESULT_ERROR				0
#define HTTPCLN_RESULT_OK					1
typedef int8_t httpcln_result_t;

typedef struct {
	uint16_t http_result_code;
	uint16_t connect_duration;
} httpcln_result_data_t;

httpcln_result_t httpcln_get(char *url, httpcln_params_t *params, httpcln_result_data_t *result_data);
httpcln_result_t httpcln_post(char *url, char *data, httpcln_params_t *params, httpcln_result_data_t *result_data);

#endif
