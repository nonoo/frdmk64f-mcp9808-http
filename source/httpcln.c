#include "httpcln.h"

#include "lwip/netdb.h"

#include <stdio.h>

#define HTTPCLN_METHOD_GET		0
#define HTTPCLN_METHOD_POST		1
typedef uint8_t httpcln_method_t;

#define HTTPCLN_WAIT_RESULT_NO_DATA_AVAILABLE		0
#define HTTPCLN_WAIT_RESULT_DATA_AVAILABLE			1
#define HTTPCLN_WAIT_RESULT_ERROR					-1
typedef int8_t httpcln_wait_result_t;

#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif

// Parses *url and places all data in the other output pointers.
static flag_t httpcln_parse_url(char *url, char *server_host, uint8_t server_host_max_size, uint16_t *port,
		char *path, uint16_t path_max_size)
{
	uint8_t url_length = strlen(url);
	uint8_t i;
	char port_str[6];
	char *endptr;

	// Setting default values.
	*port = 80;
	snprintf(path, path_max_size, "/");

	if (url_length >= 7 && strncasecmp(url, "http", 4) == 0) {
		// Skipping the http(s):// protocol definier.
		url += 4;
		while (*url != '/')
			url++;
		while (*url == '/')
			url++;
	}

	i = 0;
	while (*url != '/' && *url != ':' && *url != '?' && *url != 0) {
		if (i == server_host_max_size) { // Host did not fit?
			server_host[server_host_max_size-1] = 0;
			return 0;
		}

		server_host[i] = *url;
		i++;
		url++;
	}
	server_host[i] = 0;

	if (*url == 0) // No port and path given?
		return 1;

	if (*url == ':') { // Is there a port defined in the URL?
		url++;
		i = 0;
		while (*url != '/' && *url != '?' && *url != 0 && i < sizeof(port_str)-1) {
			port_str[i] = *url;
			i++;
			url++;
		}
		port_str[i] = 0;
		errno = 0;
		*port = strtol(port_str, &endptr, 10);
		if (errno || *endptr != 0)
			return 0;
	}

	if (path_max_size) {
		strncpy(path, url, path_max_size);
		path[path_max_size-1] = 0;
	}
	return 1;
}

static void httpcln_print_progress(uint32_t content_bytes_received, uint32_t content_length, TickType_t time_header_received_at) {
	uint32_t diff_ms;
	bool overflow;

	diff_ms = (xTaskGetTickCount()-time_header_received_at) * portTICK_PERIOD_MS;

	if (content_length)
		PRINTF("httpcln: received %u bytes (%u%% %us)\n", content_bytes_received, (uint8_t)((content_bytes_received/(float)content_length)*100.0), diff_ms/1000);
	else
		PRINTF("httpcln: received %u bytes (%us)\n", content_bytes_received, diff_ms/1000);
}

static flag_t httpcln_send(int sock, char *data, uint32_t data_len) {
	uint32_t bytes_sent;

	bytes_sent = lwip_write(sock, data, data_len);

	if (bytes_sent != data_len) {
		PRINTF("httpcln: send err\n");
		return 0;
	}

	return 1;
}

static httpcln_wait_result_t httpcln_wait_for_data_available_to_read(int sock) {
	fd_set rfds;
	fd_set efds;
	int32_t recvbytes;
	uint32_t optval;
	struct timeval tv;

	FD_ZERO(&rfds);
	FD_SET(sock, &rfds);
	FD_ZERO(&efds);
	FD_SET(sock, &efds);

	tv.tv_sec = HTTPCLN_TIMEOUT_SEC;
	tv.tv_usec = 0;

	switch (lwip_select(sock+1, &rfds, NULL, &efds, &tv)) {
		default: return HTTPCLN_WAIT_RESULT_ERROR;
		case 0: return HTTPCLN_WAIT_RESULT_NO_DATA_AVAILABLE;
		case 1:
			if (FD_ISSET(sock, &efds))
				return HTTPCLN_WAIT_RESULT_ERROR;
			return (FD_ISSET(sock, &rfds) ? HTTPCLN_WAIT_RESULT_DATA_AVAILABLE : HTTPCLN_WAIT_RESULT_NO_DATA_AVAILABLE);
	}
}

static httpcln_result_t httpcln_do(httpcln_method_t method, char *url, char *data, httpcln_params_t *params,
		httpcln_result_data_t *result_data)
{
	struct sockaddr_in server_sockaddr = {0,};
	struct hostent *hostent;
	int sock;
	uint32_t i, bytes_to_ignore;
	int received_bytes;
	uint8_t *buf = NULL;
	int bytes_in_buf = 0;
	int last_processed_byte_idx = 0;
	flag_t got_http_result_code = 0;
	flag_t header_received = 0;
	char *tok;
	flag_t chunked = 0;
	uint32_t chunk_bytes_remaining = 0;
	uint8_t chunk_header_size;
	uint32_t chunk_bytes_to_process;
	uint8_t ignore_bytes = 0;
	flag_t got_content_length_field = 0;
	flag_t got_content_length_from_range_field = 0;
	uint32_t content_length = 0;
	uint32_t content_bytes_received = 0;
	char *endptr;
	char server_host[HTTPCLN_MAX_HTTP_HOST_LENGTH];
	uint16_t port;
	char path[HTTPCLN_MAX_HTTP_URL_PATH_LENGTH];
	uint8_t zero_bytes_received_count = 0;
	flag_t result = HTTPCLN_RESULT_ERROR;
	TickType_t time_last_status_log;
	TickType_t time_started;
	bool overflow;
	uint32_t diff;
	char continue_dl_from_byte_str[11];
	char *total_length_strp;
	struct linger linger_opt = {
			.l_onoff = 1,
			.l_linger = 0
	};

	if (params->is_cancelled_cb && params->is_cancelled_cb()) {
		PRINTF("httpcln: cancelled\n");
		return HTTPCLN_RESULT_CANCELLED;
	}

	if (!httpcln_parse_url(url, server_host, sizeof(server_host), &port, path, sizeof(path))) {
		PRINTF("httpcln: invalid url\n");
		return result;
	}

	hostent = gethostbyname(server_host);
	if (!hostent)
		return result;

	memcpy(&server_sockaddr.sin_addr, hostent->h_addr, hostent->h_length);
	server_sockaddr.sin_len = hostent->h_length;
	server_sockaddr.sin_family = AF_INET;
	server_sockaddr.sin_port = htons(port);

	sock = lwip_socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		PRINTF("httpcln: socket error\n");
		goto httpcln_do_deinit;
	}

	i = HTTPCLN_TCP_WINDOW_SIZE;
	setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &i, sizeof(i));
	i = HTTPCLN_TCP_WINDOW_SIZE;
	setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &i, sizeof(i));
	i = HTTPCLN_TIMEOUT_SEC*1000;
	setsockopt(sock, SOL_SOCKET, SO_CONTIMEO, &i, sizeof(i));
	i = HTTPCLN_TIMEOUT_SEC*1000;
	setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO , &i, sizeof(i));
	i = HTTPCLN_TIMEOUT_SEC*1000;
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO , &i, sizeof(i));
	setsockopt(sock, SOL_SOCKET, SO_LINGER, &linger_opt, sizeof(linger_opt));

	i = 1000;
	setsockopt(sock, IPPROTO_TCP, TCP_KEEPALIVE, &i, sizeof(i)); // Enable keepalives.
	i = 1;
	setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &i, sizeof(i)); // Time in seconds between keepalive sends.
	i = HTTPCLN_STALL_TIMEOUT_SEC*2;
	setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &i, sizeof(i)); // Keepalive count.

	time_started = xTaskGetTickCount();
	if (lwip_connect(sock, (struct sockaddr *)&server_sockaddr, sizeof(server_sockaddr))) {
		PRINTF("httpcln: connect error\n");
		goto httpcln_do_deinit;
	}

	if (result_data)
		result_data->connect_duration = (xTaskGetTickCount()-time_started)*portTICK_PERIOD_MS;

	PRINTF("httpcln: connected to %s\n", server_host);

	if (params->is_cancelled_cb && params->is_cancelled_cb()) {
		PRINTF("httpcln: cancelled\n");
		result = HTTPCLN_RESULT_CANCELLED;
		goto httpcln_do_deinit;
	}

	// Sending the request.
	if (method == HTTPCLN_METHOD_POST) {
		if (!httpcln_send(sock, "POST ", 5))
			goto httpcln_do_deinit;
	} else {
		if (!httpcln_send(sock, "GET ", 4))
			goto httpcln_do_deinit;
	}
	if (!httpcln_send(sock, path, strlen(path)))
		goto httpcln_do_deinit;
	if (!httpcln_send(sock, " HTTP/1.1\r\nHost: ", 17))
		goto httpcln_do_deinit;
	if (!httpcln_send(sock, server_host, strlen(server_host)))
		goto httpcln_do_deinit;
	if (method == HTTPCLN_METHOD_POST) {
		if (!httpcln_send(sock, "\r\nAccept: */*\r\nContent-Length: 0\r\n", 34))
			goto httpcln_do_deinit;
	} else {
		if (!httpcln_send(sock, "\r\nConnection: close\r\nAccept-Encoding: none\r\n", 44))
			goto httpcln_do_deinit;
	}
	if (params->continue_dl_from_byte) {
		PRINTF("httpcln: req. resume from %u\n", params->continue_dl_from_byte);
		if (!httpcln_send(sock, "Range: bytes=", 13))
			goto httpcln_do_deinit;
		i = snprintf(continue_dl_from_byte_str, sizeof(continue_dl_from_byte_str), "%u", params->continue_dl_from_byte);
		if (!httpcln_send(sock, continue_dl_from_byte_str, i))
			goto httpcln_do_deinit;
		if (!httpcln_send(sock, "-\r\n", 3))
			goto httpcln_do_deinit;
	}
	for (i = 0; i < params->custom_header_entries_count; i++) {
		switch (params->custom_header_entries[i].key) {
			case HTTPCLN_HEADER_KEY_AUTH_BEARER:
				if (!httpcln_send(sock, "Authorization: Bearer ", 22))
					goto httpcln_do_deinit;
				break;
			default:
				PRINTF("httpcln: unknown hdr key\n");
				continue;
		}
		if (!httpcln_send(sock, params->custom_header_entries[i].value, strlen(params->custom_header_entries[i].value)))
			goto httpcln_do_deinit;
		if (!httpcln_send(sock, "\r\n", 2))
			goto httpcln_do_deinit;
	}
	if (!httpcln_send(sock, "\r\n", 2))
		goto httpcln_do_deinit;

	buf = (uint8_t *)malloc(HTTPCLN_TCP_WINDOW_SIZE);
	if (buf == NULL) {
		PRINTF("httpcln: malloc err\n");
		goto httpcln_do_deinit;
	}

	while (1) {
httpcln_do_need_more_bytes:
		if (params->is_cancelled_cb && params->is_cancelled_cb()) {
			PRINTF("httpcln: cancelled\n");
			result = HTTPCLN_RESULT_CANCELLED;
			goto httpcln_do_deinit;
		}

		switch (httpcln_wait_for_data_available_to_read(sock)) {
			case HTTPCLN_WAIT_RESULT_ERROR:
				PRINTF("httpcln: conn closing\n");
				goto httpcln_do_disconnected;
			case HTTPCLN_WAIT_RESULT_NO_DATA_AVAILABLE:
				zero_bytes_received_count++;
				PRINTF("httpcln: stalled for %u seconds\n", zero_bytes_received_count*HTTPCLN_TIMEOUT_SEC);
				if (zero_bytes_received_count*HTTPCLN_TIMEOUT_SEC >= HTTPCLN_STALL_TIMEOUT_SEC) {
					PRINTF("httpcln: timeout\n");
					goto httpcln_do_deinit;
				}
				continue;
			default:
				zero_bytes_received_count = 0;
				break;
		}

		errno = 0;
		received_bytes = recv(sock, buf+bytes_in_buf, HTTPCLN_TCP_WINDOW_SIZE-bytes_in_buf, 0);

		if (received_bytes <= 0) {
			if (errno == ENOTCONN) {
				PRINTF("httpcln: conn closing\n");
				goto httpcln_do_disconnected;
			}
			continue;
		}

		bytes_in_buf += received_bytes;
		if (!header_received) {
			while (received_bytes--) {
				// Ignoring \r in response header.
				if (buf[last_processed_byte_idx] == '\r')
					buf[last_processed_byte_idx] = 0;
				else if (buf[last_processed_byte_idx] == '\n') {
					buf[last_processed_byte_idx] = 0;

					if (buf[0] == 0) { // Header is complete when an empty line is received.
						if (!got_http_result_code) {
							PRINTF("httpcln: got no result code\n");
							goto httpcln_do_deinit;
						}

						// Removing the \r\n chars.
						bytes_in_buf -= last_processed_byte_idx+1;
						for (i = 0; i < bytes_in_buf; i++)
							buf[i] = buf[i+last_processed_byte_idx+1];

						params->time_header_received_at = xTaskGetTickCount();
						header_received = 1;
						if (chunked) {
							// We don't know the content length, but we keep it zero here for security reasons as we only
							// use the content length value given in the params for displaying the progress (it may be
							// different than the real chunked content length).
							content_length = 0;

							if (params->content_length_for_progress)
								PRINTF("httpcln: chunked transfer, using %u as content length\n", params->content_length_for_progress);
							else
								PRINTF("httpcln: chunked transfer\n");
						} else if (got_content_length_field) {
							PRINTF("httpcln: content-length: %u bytes\n", content_length);

							if (content_length == 0) {
								result = HTTPCLN_RESULT_OK;
								goto httpcln_do_deinit;
							}

							if (params->max_resp_content_length && content_length > params->max_resp_content_length) {
								PRINTF("httpcln: max. content length is %u, abort\n", params->max_resp_content_length);
								goto httpcln_do_deinit;
							}
						} else
							PRINTF("httpcln: no content-length field, using %u as content length\n", params->content_length_for_progress);
						break;
					}

					tok = strtok((char *)buf, " ");
					if (memcmp(tok, "HTTP/1.1", 7) == 0) {
						tok = strtok(NULL, " ");
						errno = 0;
						i = strtol(tok, &endptr, 10);
						if (*endptr != 0 || errno) {
							PRINTF("httpcln: error %s\n", tok);
							goto httpcln_do_deinit;
						}

						got_http_result_code = 1;
						if (result_data)
							result_data->http_result_code = i;

						switch (i) {
							case 401:
								result = HTTPCLN_RESULT_UNAUTHORIZED;
								PRINTF("httpcln: unauthorized\n");
								goto httpcln_do_deinit;
							case 405:
								result = HTTPCLN_RESULT_UNALLOWED;
								PRINTF("httpcln: unallowed\n");
								goto httpcln_do_deinit;
							default:
								if (i < 200 || i > 299) {
									PRINTF("httpcln: error %u\n", i);
									goto httpcln_do_deinit;
								}
						}
					}
					if (last_processed_byte_idx > 16) {
						if (strncasecmp(tok, "Content-Length:", 15) == 0) {
							got_content_length_field = 1;

							if (!got_content_length_from_range_field) {
								errno = 0;
								content_length = strtoul((char *)buf+16, &endptr, 10);
								if (*endptr != 0 || errno)
									content_length = 0;
							}
						}
						if (last_processed_byte_idx > 21) {
							if (strncasecmp(tok, "Content-Range:", 14) == 0 && strncasecmp((char *)buf+15, "bytes", 5) == 0) {
								errno = 0;
								content_bytes_received = strtoul((char *)buf+21, &endptr, 10);
								if ((*endptr != '-' && *endptr != '/' && *endptr != 0) || errno) {
									PRINTF("httpcln: resume dl error\n");
									goto httpcln_do_deinit;
								}

								total_length_strp = strrchr((char *)buf+21, '/')+1;
								if (total_length_strp && *total_length_strp != '*') {
									errno = 0;
									content_length = strtoul(total_length_strp, &endptr, 10);
									if (*endptr != 0 || errno) {
										PRINTF("httpcln: resume dl error\n");
										goto httpcln_do_deinit;
									}
								}

								got_content_length_from_range_field = 1;

								PRINTF("httpcln: resuming from %u\n", content_bytes_received);
							}

							if (last_processed_byte_idx > 26) {
								if (strncasecmp(tok, "Transfer-Encoding:", 18) == 0 && strncasecmp((char *)buf+19, "chunked", 7) == 0)
									chunked = 1;
							}
						}
					}
					bytes_in_buf -= last_processed_byte_idx+1;
					for (i = 0; i < bytes_in_buf; i++)
						buf[i] = buf[i+last_processed_byte_idx+1];
					last_processed_byte_idx = -1;
				}

				last_processed_byte_idx++;
			}
		}

		if (header_received) {
			if (chunked) {
				while (bytes_in_buf) {
					if (ignore_bytes) {
						bytes_to_ignore = min(bytes_in_buf, ignore_bytes);
						ignore_bytes -= bytes_to_ignore;
						bytes_in_buf -= bytes_to_ignore;
						for (i = 0; i < bytes_in_buf; i++)
							buf[i] = buf[i+bytes_to_ignore];

						if (ignore_bytes)
							goto httpcln_do_need_more_bytes;
					}

					if (chunk_bytes_remaining == 0) { // Reading the chunk header first
						if (bytes_in_buf < 3) // We need at least 1 length byte and 2 line end bytes (\r\n).
							goto httpcln_do_need_more_bytes;

						for (i = 1; i < bytes_in_buf; i++) {
							if (buf[i-1] == '\r' && buf[i] == '\n')
								break;
						}
						if (i == bytes_in_buf) // No \r\n found?
							goto httpcln_do_need_more_bytes;

						chunk_header_size = i+1;

						errno = 0;
						chunk_bytes_remaining = strtoul((char *)buf, &endptr, 16);
						if ((*endptr != '\r' && *endptr != ';') || errno) {
							PRINTF("httpcln: invalid chunk length\n");
							goto httpcln_do_deinit;
						}

						content_length += chunk_bytes_remaining;
						PRINTF("httpcln: got %u byte chunk (total %u bytes)\n", chunk_bytes_remaining, content_length);

						if (chunk_bytes_remaining == 0) { // Chunked transfer is over.
							httpcln_print_progress(content_bytes_received, params->content_length_for_progress ?
									params->content_length_for_progress : content_length, params->time_header_received_at);
							result = HTTPCLN_RESULT_OK;
							goto httpcln_do_deinit;
						}

						if (content_length > params->max_resp_content_length) {
							PRINTF("httpcln: max. content length is %u, abort\n", params->max_resp_content_length);
							goto httpcln_do_deinit;
						}

						// Cutting the chunk header off.
						bytes_in_buf -= chunk_header_size;
						for (i = 0; i < bytes_in_buf; i++)
							buf[i] = buf[i+chunk_header_size];
					}

					chunk_bytes_to_process = min(chunk_bytes_remaining, bytes_in_buf);
					content_bytes_received += chunk_bytes_to_process;
					if (content_bytes_received > params->max_resp_content_length) {
						PRINTF("httpcln: max. content length is %u, abort\n", params->max_resp_content_length);
						goto httpcln_do_deinit;
					}

					if (params->processdata_cb && !params->processdata_cb(buf, chunk_bytes_to_process,
							params->content_length_for_progress ? params->content_length_for_progress : content_length,
							content_bytes_received, params->processdata_cb_param))
						goto httpcln_do_deinit;

					chunk_bytes_remaining -= chunk_bytes_to_process;
					if (chunk_bytes_remaining == 0)
						ignore_bytes = 2; // Ignoring \r\n after the chunk.
					bytes_in_buf -= chunk_bytes_to_process;
					for (i = 0; i < bytes_in_buf; i++)
						buf[i] = buf[i+chunk_bytes_to_process];

					if ((xTaskGetTickCount()-time_last_status_log)*portTICK_PERIOD_MS >= 2000 || overflow) {
						httpcln_print_progress(content_bytes_received, params->content_length_for_progress ?
								params->content_length_for_progress : content_length, params->time_header_received_at);
						time_last_status_log = xTaskGetTickCount();
					}
				}
			} else {
				content_bytes_received += bytes_in_buf;

				if (content_bytes_received > params->max_resp_content_length) {
					PRINTF("httpcln: max. content length is %u, abort\n", params->max_resp_content_length);
					goto httpcln_do_deinit;
				}

				if ((!got_content_length_field || content_bytes_received < content_length) &&
						(xTaskGetTickCount()-time_last_status_log)*portTICK_PERIOD_MS >= 2000)
				{
					httpcln_print_progress(content_bytes_received, params->content_length_for_progress ? params->content_length_for_progress : content_length, params->time_header_received_at);
					time_last_status_log = xTaskGetTickCount();
				}

				if (params->processdata_cb && !params->processdata_cb(buf, bytes_in_buf,
						params->content_length_for_progress ? params->content_length_for_progress : content_length, content_bytes_received,
						params->processdata_cb_param))
					goto httpcln_do_deinit;

				if (got_content_length_field && content_bytes_received >= content_length) {
					httpcln_print_progress(content_bytes_received,
							params->content_length_for_progress ? params->content_length_for_progress : content_length, params->time_header_received_at);
					result = HTTPCLN_RESULT_OK;
					goto httpcln_do_deinit;
				}
			}

			bytes_in_buf = 0;
		}
	}

	goto httpcln_do_deinit;

httpcln_do_disconnected:
	if (header_received && !got_content_length_field)
		result = HTTPCLN_RESULT_OK;
	PRINTF("httpcln: disconnected\n");

httpcln_do_deinit:
	if (sock >= 0)
		lwip_close(sock);

	if (buf)
		free(buf);

	return result;
}

// Received HTTP data and the given processdata_param is passed to the processdata() callback function.
// If the callback function returns 0, the connection is terminated.
// Set max_content_length to 0 if you don't want any restrictions on max. content length.
httpcln_result_t httpcln_get(char *url, httpcln_params_t *params, httpcln_result_data_t *result_data) {
	PRINTF("httpcln: get %s\n", url);
	return httpcln_do(HTTPCLN_METHOD_GET, url, NULL, params, result_data);
}

// NOTE: posting *data is not implemented yet!
// Received HTTP data and the given processdata_param is passed to the processdata() callback function.
// If the callback function returns 0, the connection is terminated.
// Set max_content_length to 0 if you don't want any restrictions on max. content length.
httpcln_result_t httpcln_post(char *url, char *data, httpcln_params_t *params, httpcln_result_data_t *result_data) {
	PRINTF("httpcln: post %s\n", url);
	return httpcln_do(HTTPCLN_METHOD_POST, url, data, params, result_data);
}
