/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.9 */

#ifndef PB_MESHTASTIC_MESHTASTIC_ATAK_PB_H_INCLUDED
#define PB_MESHTASTIC_MESHTASTIC_ATAK_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _meshtastic_Team {
    /* Unspecifed */
    meshtastic_Team_Unspecifed_Color = 0,
    /* White */
    meshtastic_Team_White = 1,
    /* Yellow */
    meshtastic_Team_Yellow = 2,
    /* Orange */
    meshtastic_Team_Orange = 3,
    /* Magenta */
    meshtastic_Team_Magenta = 4,
    /* Red */
    meshtastic_Team_Red = 5,
    /* Maroon */
    meshtastic_Team_Maroon = 6,
    /* Purple */
    meshtastic_Team_Purple = 7,
    /* Dark Blue */
    meshtastic_Team_Dark_Blue = 8,
    /* Blue */
    meshtastic_Team_Blue = 9,
    /* Cyan */
    meshtastic_Team_Cyan = 10,
    /* Teal */
    meshtastic_Team_Teal = 11,
    /* Green */
    meshtastic_Team_Green = 12,
    /* Dark Green */
    meshtastic_Team_Dark_Green = 13,
    /* Brown */
    meshtastic_Team_Brown = 14
} meshtastic_Team;

/* Role of the group member */
typedef enum _meshtastic_MemberRole {
    /* Unspecifed */
    meshtastic_MemberRole_Unspecifed = 0,
    /* Team Member */
    meshtastic_MemberRole_TeamMember = 1,
    /* Team Lead */
    meshtastic_MemberRole_TeamLead = 2,
    /* Headquarters */
    meshtastic_MemberRole_HQ = 3,
    /* Airsoft enthusiast */
    meshtastic_MemberRole_Sniper = 4,
    /* Medic */
    meshtastic_MemberRole_Medic = 5,
    /* ForwardObserver */
    meshtastic_MemberRole_ForwardObserver = 6,
    /* Radio Telephone Operator */
    meshtastic_MemberRole_RTO = 7,
    /* Doggo */
    meshtastic_MemberRole_K9 = 8
} meshtastic_MemberRole;

/* Struct definitions */
/* ATAK GeoChat message */
typedef struct _meshtastic_GeoChat {
    /* The text message */
    char message[200];
    /* Uid recipient of the message */
    bool has_to;
    char to[120];
    /* Callsign of the recipient for the message */
    bool has_to_callsign;
    char to_callsign[120];
} meshtastic_GeoChat;

/* ATAK Group
 <__group role='Team Member' name='Cyan'/> */
typedef struct _meshtastic_Group {
    /* Role of the group member */
    meshtastic_MemberRole role;
    /* Team (color)
 Default Cyan */
    meshtastic_Team team;
} meshtastic_Group;

/* ATAK EUD Status
 <status battery='100' /> */
typedef struct _meshtastic_Status {
    /* Battery level */
    uint8_t battery;
} meshtastic_Status;

/* ATAK Contact
 <contact endpoint='0.0.0.0:4242:tcp' phone='+12345678' callsign='FALKE'/> */
typedef struct _meshtastic_Contact {
    /* Callsign */
    char callsign[120];
    /* Device callsign */
    char device_callsign[120]; /* IP address of endpoint in integer form (0.0.0.0 default) */
} meshtastic_Contact;

/* Position Location Information from ATAK */
typedef struct _meshtastic_PLI {
    /* The new preferred location encoding, multiply by 1e-7 to get degrees
 in floating point */
    int32_t latitude_i;
    /* The new preferred location encoding, multiply by 1e-7 to get degrees
 in floating point */
    int32_t longitude_i;
    /* Altitude (ATAK prefers HAE) */
    int32_t altitude;
    /* Speed */
    uint32_t speed;
    /* Course in degrees */
    uint16_t course;
} meshtastic_PLI;

typedef PB_BYTES_ARRAY_T(220) meshtastic_TAKPacket_detail_t;
/* Packets for the official ATAK Plugin */
typedef struct _meshtastic_TAKPacket {
    /* Are the payloads strings compressed for LoRA transport? */
    bool is_compressed;
    /* The contact / callsign for ATAK user */
    bool has_contact;
    meshtastic_Contact contact;
    /* The group for ATAK user */
    bool has_group;
    meshtastic_Group group;
    /* The status of the ATAK EUD */
    bool has_status;
    meshtastic_Status status;
    pb_size_t which_payload_variant;
    union {
        /* TAK position report */
        meshtastic_PLI pli;
        /* ATAK GeoChat message */
        meshtastic_GeoChat chat;
        /* Generic CoT detail XML
     May be compressed / truncated by the sender (EUD) */
        meshtastic_TAKPacket_detail_t detail;
    } payload_variant;
} meshtastic_TAKPacket;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _meshtastic_Team_MIN meshtastic_Team_Unspecifed_Color
#define _meshtastic_Team_MAX meshtastic_Team_Brown
#define _meshtastic_Team_ARRAYSIZE ((meshtastic_Team)(meshtastic_Team_Brown+1))

#define _meshtastic_MemberRole_MIN meshtastic_MemberRole_Unspecifed
#define _meshtastic_MemberRole_MAX meshtastic_MemberRole_K9
#define _meshtastic_MemberRole_ARRAYSIZE ((meshtastic_MemberRole)(meshtastic_MemberRole_K9+1))



#define meshtastic_Group_role_ENUMTYPE meshtastic_MemberRole
#define meshtastic_Group_team_ENUMTYPE meshtastic_Team





/* Initializer values for message structs */
#define meshtastic_TAKPacket_init_default        {0, false, meshtastic_Contact_init_default, false, meshtastic_Group_init_default, false, meshtastic_Status_init_default, 0, {meshtastic_PLI_init_default}}
#define meshtastic_GeoChat_init_default          {"", false, "", false, ""}
#define meshtastic_Group_init_default            {_meshtastic_MemberRole_MIN, _meshtastic_Team_MIN}
#define meshtastic_Status_init_default           {0}
#define meshtastic_Contact_init_default          {"", ""}
#define meshtastic_PLI_init_default              {0, 0, 0, 0, 0}
#define meshtastic_TAKPacket_init_zero           {0, false, meshtastic_Contact_init_zero, false, meshtastic_Group_init_zero, false, meshtastic_Status_init_zero, 0, {meshtastic_PLI_init_zero}}
#define meshtastic_GeoChat_init_zero             {"", false, "", false, ""}
#define meshtastic_Group_init_zero               {_meshtastic_MemberRole_MIN, _meshtastic_Team_MIN}
#define meshtastic_Status_init_zero              {0}
#define meshtastic_Contact_init_zero             {"", ""}
#define meshtastic_PLI_init_zero                 {0, 0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define meshtastic_GeoChat_message_tag           1
#define meshtastic_GeoChat_to_tag                2
#define meshtastic_GeoChat_to_callsign_tag       3
#define meshtastic_Group_role_tag                1
#define meshtastic_Group_team_tag                2
#define meshtastic_Status_battery_tag            1
#define meshtastic_Contact_callsign_tag          1
#define meshtastic_Contact_device_callsign_tag   2
#define meshtastic_PLI_latitude_i_tag            1
#define meshtastic_PLI_longitude_i_tag           2
#define meshtastic_PLI_altitude_tag              3
#define meshtastic_PLI_speed_tag                 4
#define meshtastic_PLI_course_tag                5
#define meshtastic_TAKPacket_is_compressed_tag   1
#define meshtastic_TAKPacket_contact_tag         2
#define meshtastic_TAKPacket_group_tag           3
#define meshtastic_TAKPacket_status_tag          4
#define meshtastic_TAKPacket_pli_tag             5
#define meshtastic_TAKPacket_chat_tag            6
#define meshtastic_TAKPacket_detail_tag          7

/* Struct field encoding specification for nanopb */
#define meshtastic_TAKPacket_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     is_compressed,     1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  contact,           2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  group,             3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  status,            4) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload_variant,pli,payload_variant.pli),   5) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload_variant,chat,payload_variant.chat),   6) \
X(a, STATIC,   ONEOF,    BYTES,    (payload_variant,detail,payload_variant.detail),   7)
#define meshtastic_TAKPacket_CALLBACK NULL
#define meshtastic_TAKPacket_DEFAULT NULL
#define meshtastic_TAKPacket_contact_MSGTYPE meshtastic_Contact
#define meshtastic_TAKPacket_group_MSGTYPE meshtastic_Group
#define meshtastic_TAKPacket_status_MSGTYPE meshtastic_Status
#define meshtastic_TAKPacket_payload_variant_pli_MSGTYPE meshtastic_PLI
#define meshtastic_TAKPacket_payload_variant_chat_MSGTYPE meshtastic_GeoChat

#define meshtastic_GeoChat_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, STRING,   message,           1) \
X(a, STATIC,   OPTIONAL, STRING,   to,                2) \
X(a, STATIC,   OPTIONAL, STRING,   to_callsign,       3)
#define meshtastic_GeoChat_CALLBACK NULL
#define meshtastic_GeoChat_DEFAULT NULL

#define meshtastic_Group_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UENUM,    role,              1) \
X(a, STATIC,   SINGULAR, UENUM,    team,              2)
#define meshtastic_Group_CALLBACK NULL
#define meshtastic_Group_DEFAULT NULL

#define meshtastic_Status_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT32,   battery,           1)
#define meshtastic_Status_CALLBACK NULL
#define meshtastic_Status_DEFAULT NULL

#define meshtastic_Contact_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, STRING,   callsign,          1) \
X(a, STATIC,   SINGULAR, STRING,   device_callsign,   2)
#define meshtastic_Contact_CALLBACK NULL
#define meshtastic_Contact_DEFAULT NULL

#define meshtastic_PLI_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, SFIXED32, latitude_i,        1) \
X(a, STATIC,   SINGULAR, SFIXED32, longitude_i,       2) \
X(a, STATIC,   SINGULAR, INT32,    altitude,          3) \
X(a, STATIC,   SINGULAR, UINT32,   speed,             4) \
X(a, STATIC,   SINGULAR, UINT32,   course,            5)
#define meshtastic_PLI_CALLBACK NULL
#define meshtastic_PLI_DEFAULT NULL

extern const pb_msgdesc_t meshtastic_TAKPacket_msg;
extern const pb_msgdesc_t meshtastic_GeoChat_msg;
extern const pb_msgdesc_t meshtastic_Group_msg;
extern const pb_msgdesc_t meshtastic_Status_msg;
extern const pb_msgdesc_t meshtastic_Contact_msg;
extern const pb_msgdesc_t meshtastic_PLI_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define meshtastic_TAKPacket_fields &meshtastic_TAKPacket_msg
#define meshtastic_GeoChat_fields &meshtastic_GeoChat_msg
#define meshtastic_Group_fields &meshtastic_Group_msg
#define meshtastic_Status_fields &meshtastic_Status_msg
#define meshtastic_Contact_fields &meshtastic_Contact_msg
#define meshtastic_PLI_fields &meshtastic_PLI_msg

/* Maximum encoded size of messages (where known) */
#define MESHTASTIC_MESHTASTIC_ATAK_PB_H_MAX_SIZE meshtastic_TAKPacket_size
#define meshtastic_Contact_size                  242
#define meshtastic_GeoChat_size                  444
#define meshtastic_Group_size                    4
#define meshtastic_PLI_size                      31
#define meshtastic_Status_size                   3
#define meshtastic_TAKPacket_size                705

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
