/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_ROBOTMSGS_PB_H_INCLUDED
#define PB_ROBOTMSGS_PB_H_INCLUDED
#include "pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _RobotSensors {
  float lead_screw_curr;
  float act_right_curr;
  float dep_curr;
  float exc_curr;
  float drive_left_curr;
  float drive_right_curr;
  float drive_left_ang;
  float drive_right_ang;
  float exc_ang;
  float uwb_dist_0;
  float uwb_dist_1;
  float uwb_dist_2;
} RobotSensors;

typedef struct _RobotEffort {
  int32_t lin_act;
  int32_t left_drive;
  int32_t right_drive;
  int32_t excavate;
  int32_t deposit;
} RobotEffort;

#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define RobotSensors_init_default                                                                  \
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define RobotEffort_init_default                                                                   \
  { 0, 0, 0, 0, 0 }
#define RobotSensors_init_zero                                                                     \
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
#define RobotEffort_init_zero                                                                      \
  { 0, 0, 0, 0, 0 }

/* Field tags (for use in manual encoding/decoding) */
#define RobotSensors_lead_screw_curr_tag 1
#define RobotSensors_act_right_curr_tag 2
#define RobotSensors_dep_curr_tag 3
#define RobotSensors_exc_curr_tag 4
#define RobotSensors_drive_left_curr_tag 5
#define RobotSensors_drive_right_curr_tag 6
#define RobotSensors_drive_left_ang_tag 7
#define RobotSensors_drive_right_ang_tag 8
#define RobotSensors_exc_ang_tag 9
#define RobotSensors_uwb_dist_0_tag 10
#define RobotSensors_uwb_dist_1_tag 11
#define RobotSensors_uwb_dist_2_tag 12
#define RobotEffort_lin_act_tag 1
#define RobotEffort_left_drive_tag 2
#define RobotEffort_right_drive_tag 3
#define RobotEffort_excavate_tag 4
#define RobotEffort_deposit_tag 5

/* Struct field encoding specification for nanopb */
#define RobotSensors_FIELDLIST(X, a)                                                               \
  X(a, STATIC, SINGULAR, SINT32, lead_screw_curr, 1)                                               \
  X(a, STATIC, SINGULAR, SINT32, act_right_curr, 2)                                                \
  X(a, STATIC, SINGULAR, SINT32, dep_curr, 3)                                                      \
  X(a, STATIC, SINGULAR, SINT32, exc_curr, 4)                                                      \
  X(a, STATIC, SINGULAR, SINT32, drive_left_curr, 5)                                               \
  X(a, STATIC, SINGULAR, SINT32, drive_right_curr, 6)                                              \
  X(a, STATIC, SINGULAR, FLOAT, drive_left_ang, 7)                                                 \
  X(a, STATIC, SINGULAR, FLOAT, drive_right_ang, 8)                                                \
  X(a, STATIC, SINGULAR, FLOAT, exc_ang, 9)                                                        \
  X(a, STATIC, SINGULAR, FLOAT, uwb_dist_0, 10)                                                    \
  X(a, STATIC, SINGULAR, FLOAT, uwb_dist_1, 11)                                                    \
  X(a, STATIC, SINGULAR, FLOAT, uwb_dist_2, 12)
#define RobotSensors_CALLBACK NULL
#define RobotSensors_DEFAULT NULL

#define RobotEffort_FIELDLIST(X, a)                                                                \
  X(a, STATIC, SINGULAR, SINT32, lin_act, 1)                                                       \
  X(a, STATIC, SINGULAR, SINT32, left_drive, 2)                                                    \
  X(a, STATIC, SINGULAR, SINT32, right_drive, 3)                                                   \
  X(a, STATIC, SINGULAR, SINT32, excavate, 4)                                                      \
  X(a, STATIC, SINGULAR, SINT32, deposit, 5)
#define RobotEffort_CALLBACK NULL
#define RobotEffort_DEFAULT NULL

extern const pb_msgdesc_t RobotSensors_msg;
extern const pb_msgdesc_t RobotEffort_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define RobotSensors_fields &RobotSensors_msg
#define RobotEffort_fields &RobotEffort_msg

/* Maximum encoded size of messages (where known) */
#define RobotEffort_size 36
#define RobotSensors_size 81

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
