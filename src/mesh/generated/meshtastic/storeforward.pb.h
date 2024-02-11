/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_MESHTASTIC_MESHTASTIC_STOREFORWARD_PB_H_INCLUDED
#define PB_MESHTASTIC_MESHTASTIC_STOREFORWARD_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
/* 001 - 063 = From Router
 064 - 127 = From Client */
typedef enum _meshtastic_StoreAndForward_RequestResponse {
    /* Unset/unused */
    meshtastic_StoreAndForward_RequestResponse_UNSET = 0,
    /* Router is an in error state. */
    meshtastic_StoreAndForward_RequestResponse_ROUTER_ERROR = 1,
    /* Router heartbeat */
    meshtastic_StoreAndForward_RequestResponse_ROUTER_HEARTBEAT = 2,
    /* Router has requested the client respond. This can work as a
 "are you there" message. */
    meshtastic_StoreAndForward_RequestResponse_ROUTER_PING = 3,
    /* The response to a "Ping" */
    meshtastic_StoreAndForward_RequestResponse_ROUTER_PONG = 4,
    /* Router is currently busy. Please try again later. */
    meshtastic_StoreAndForward_RequestResponse_ROUTER_BUSY = 5,
    /* Router is responding to a request for history. */
    meshtastic_StoreAndForward_RequestResponse_ROUTER_HISTORY = 6,
    /* Router is responding to a request for stats. */
    meshtastic_StoreAndForward_RequestResponse_ROUTER_STATS = 7,
    /* Client is an in error state. */
    meshtastic_StoreAndForward_RequestResponse_CLIENT_ERROR = 64,
    /* Client has requested a replay from the router. */
    meshtastic_StoreAndForward_RequestResponse_CLIENT_HISTORY = 65,
    /* Client has requested stats from the router. */
    meshtastic_StoreAndForward_RequestResponse_CLIENT_STATS = 66,
    /* Client has requested the router respond. This can work as a
 "are you there" message. */
    meshtastic_StoreAndForward_RequestResponse_CLIENT_PING = 67,
    /* The response to a "Ping" */
    meshtastic_StoreAndForward_RequestResponse_CLIENT_PONG = 68,
    /* Client has requested that the router abort processing the client's request */
    meshtastic_StoreAndForward_RequestResponse_CLIENT_ABORT = 106
} meshtastic_StoreAndForward_RequestResponse;

/* Struct definitions */
/* TODO: REPLACE */
typedef struct _meshtastic_StoreAndForward_Statistics {
    /* Number of messages we have ever seen */
    uint32_t messages_total;
    /* Number of messages we have currently saved our history. */
    uint32_t messages_saved;
    /* Maximum number of messages we will save */
    uint32_t messages_max;
    /* Router uptime in seconds */
    uint32_t up_time;
    /* Number of times any client sent a request to the S&F. */
    uint32_t requests;
    /* Number of times the history was requested. */
    uint32_t requests_history;
    /* Is the heartbeat enabled on the server? */
    bool heartbeat;
    /* Maximum number of messages the server will return. */
    uint32_t return_max;
    /* Maximum history window in minutes the server will return messages from. */
    uint32_t return_window;
} meshtastic_StoreAndForward_Statistics;

/* TODO: REPLACE */
typedef struct _meshtastic_StoreAndForward_History {
    /* Number of that will be sent to the client */
    uint32_t history_messages;
    /* The window of messages that was used to filter the history client requested */
    uint32_t window;
    /* Index in the packet history of the last message sent in a previous request to the server.
 Will be sent to the client before sending the history and can be set in a subsequent request to avoid getting packets the server already sent to the client. */
    uint32_t last_request;
} meshtastic_StoreAndForward_History;

/* TODO: REPLACE */
typedef struct _meshtastic_StoreAndForward_Heartbeat {
    /* Period in seconds that the heartbeat is sent out that will be sent to the client */
    uint32_t period;
    /* If set, this is not the primary Store & Forward router on the mesh */
    uint32_t secondary;
} meshtastic_StoreAndForward_Heartbeat;

typedef PB_BYTES_ARRAY_T(237) meshtastic_StoreAndForward_text_t;
/* TODO: REPLACE */
typedef struct _meshtastic_StoreAndForward {
    /* TODO: REPLACE */
    meshtastic_StoreAndForward_RequestResponse rr;
    pb_size_t which_variant;
    union {
        /* TODO: REPLACE */
        meshtastic_StoreAndForward_Statistics stats;
        /* TODO: REPLACE */
        meshtastic_StoreAndForward_History history;
        /* TODO: REPLACE */
        meshtastic_StoreAndForward_Heartbeat heartbeat;
        /* Text from history message. */
        meshtastic_StoreAndForward_text_t text;
    } variant;
} meshtastic_StoreAndForward;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _meshtastic_StoreAndForward_RequestResponse_MIN meshtastic_StoreAndForward_RequestResponse_UNSET
#define _meshtastic_StoreAndForward_RequestResponse_MAX meshtastic_StoreAndForward_RequestResponse_CLIENT_ABORT
#define _meshtastic_StoreAndForward_RequestResponse_ARRAYSIZE ((meshtastic_StoreAndForward_RequestResponse)(meshtastic_StoreAndForward_RequestResponse_CLIENT_ABORT+1))

#define meshtastic_StoreAndForward_rr_ENUMTYPE meshtastic_StoreAndForward_RequestResponse





/* Initializer values for message structs */
#define meshtastic_StoreAndForward_init_default  {_meshtastic_StoreAndForward_RequestResponse_MIN, 0, {meshtastic_StoreAndForward_Statistics_init_default}}
#define meshtastic_StoreAndForward_Statistics_init_default {0, 0, 0, 0, 0, 0, 0, 0, 0}
#define meshtastic_StoreAndForward_History_init_default {0, 0, 0}
#define meshtastic_StoreAndForward_Heartbeat_init_default {0, 0}
#define meshtastic_StoreAndForward_init_zero     {_meshtastic_StoreAndForward_RequestResponse_MIN, 0, {meshtastic_StoreAndForward_Statistics_init_zero}}
#define meshtastic_StoreAndForward_Statistics_init_zero {0, 0, 0, 0, 0, 0, 0, 0, 0}
#define meshtastic_StoreAndForward_History_init_zero {0, 0, 0}
#define meshtastic_StoreAndForward_Heartbeat_init_zero {0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define meshtastic_StoreAndForward_Statistics_messages_total_tag 1
#define meshtastic_StoreAndForward_Statistics_messages_saved_tag 2
#define meshtastic_StoreAndForward_Statistics_messages_max_tag 3
#define meshtastic_StoreAndForward_Statistics_up_time_tag 4
#define meshtastic_StoreAndForward_Statistics_requests_tag 5
#define meshtastic_StoreAndForward_Statistics_requests_history_tag 6
#define meshtastic_StoreAndForward_Statistics_heartbeat_tag 7
#define meshtastic_StoreAndForward_Statistics_return_max_tag 8
#define meshtastic_StoreAndForward_Statistics_return_window_tag 9
#define meshtastic_StoreAndForward_History_history_messages_tag 1
#define meshtastic_StoreAndForward_History_window_tag 2
#define meshtastic_StoreAndForward_History_last_request_tag 3
#define meshtastic_StoreAndForward_Heartbeat_period_tag 1
#define meshtastic_StoreAndForward_Heartbeat_secondary_tag 2
#define meshtastic_StoreAndForward_rr_tag        1
#define meshtastic_StoreAndForward_stats_tag     2
#define meshtastic_StoreAndForward_history_tag   3
#define meshtastic_StoreAndForward_heartbeat_tag 4
#define meshtastic_StoreAndForward_text_tag      5

/* Struct field encoding specification for nanopb */
#define meshtastic_StoreAndForward_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UENUM,    rr,                1) \
X(a, STATIC,   ONEOF,    MESSAGE,  (variant,stats,variant.stats),   2) \
X(a, STATIC,   ONEOF,    MESSAGE,  (variant,history,variant.history),   3) \
X(a, STATIC,   ONEOF,    MESSAGE,  (variant,heartbeat,variant.heartbeat),   4) \
X(a, STATIC,   ONEOF,    BYTES,    (variant,text,variant.text),   5)
#define meshtastic_StoreAndForward_CALLBACK NULL
#define meshtastic_StoreAndForward_DEFAULT NULL
#define meshtastic_StoreAndForward_variant_stats_MSGTYPE meshtastic_StoreAndForward_Statistics
#define meshtastic_StoreAndForward_variant_history_MSGTYPE meshtastic_StoreAndForward_History
#define meshtastic_StoreAndForward_variant_heartbeat_MSGTYPE meshtastic_StoreAndForward_Heartbeat

#define meshtastic_StoreAndForward_Statistics_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   messages_total,    1) \
X(a, STATIC,   SINGULAR, UINT32,   messages_saved,    2) \
X(a, STATIC,   SINGULAR, UINT32,   messages_max,      3) \
X(a, STATIC,   SINGULAR, UINT32,   up_time,           4) \
X(a, STATIC,   SINGULAR, UINT32,   requests,          5) \
X(a, STATIC,   SINGULAR, UINT32,   requests_history,   6) \
X(a, STATIC,   SINGULAR, BOOL,     heartbeat,         7) \
X(a, STATIC,   SINGULAR, UINT32,   return_max,        8) \
X(a, STATIC,   SINGULAR, UINT32,   return_window,     9)
#define meshtastic_StoreAndForward_Statistics_CALLBACK NULL
#define meshtastic_StoreAndForward_Statistics_DEFAULT NULL

#define meshtastic_StoreAndForward_History_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   history_messages,   1) \
X(a, STATIC,   SINGULAR, UINT32,   window,            2) \
X(a, STATIC,   SINGULAR, UINT32,   last_request,      3)
#define meshtastic_StoreAndForward_History_CALLBACK NULL
#define meshtastic_StoreAndForward_History_DEFAULT NULL

#define meshtastic_StoreAndForward_Heartbeat_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   period,            1) \
X(a, STATIC,   SINGULAR, UINT32,   secondary,         2)
#define meshtastic_StoreAndForward_Heartbeat_CALLBACK NULL
#define meshtastic_StoreAndForward_Heartbeat_DEFAULT NULL

extern const pb_msgdesc_t meshtastic_StoreAndForward_msg;
extern const pb_msgdesc_t meshtastic_StoreAndForward_Statistics_msg;
extern const pb_msgdesc_t meshtastic_StoreAndForward_History_msg;
extern const pb_msgdesc_t meshtastic_StoreAndForward_Heartbeat_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define meshtastic_StoreAndForward_fields &meshtastic_StoreAndForward_msg
#define meshtastic_StoreAndForward_Statistics_fields &meshtastic_StoreAndForward_Statistics_msg
#define meshtastic_StoreAndForward_History_fields &meshtastic_StoreAndForward_History_msg
#define meshtastic_StoreAndForward_Heartbeat_fields &meshtastic_StoreAndForward_Heartbeat_msg

/* Maximum encoded size of messages (where known) */
#define meshtastic_StoreAndForward_Heartbeat_size 12
#define meshtastic_StoreAndForward_History_size  18
#define meshtastic_StoreAndForward_Statistics_size 50
#define meshtastic_StoreAndForward_size          242

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
