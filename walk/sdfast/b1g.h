/************************************************************************/
/* SDFAST constants for "in air" model b1g */

/* Sizes */
#define NQ 7
#define NU 7
#define NSTATE (NQ+NU)

/* Indices for sdfast state */
#define Q_X 0
#define Q_Z 1
#define Q_PITCH 2
#define Q_L_HIP 3
#define Q_L_KNEE 4
#define Q_R_HIP 5
#define Q_R_KNEE 6

#define QD_X 7
#define QD_Z 8
#define QD_PITCH 9
#define QD_L_HIP 10
#define QD_L_KNEE 11
#define QD_R_HIP 12
#define QD_R_KNEE 13

// Body index definitions (for sdpointf, etc.)
#define BODY_WORLD -1
#define BODY_TORSO 0
#define BODY_L_THIGH 1
#define BODY_L_CALF 2
#define BODY_R_THIGH 3
#define BODY_R_CALF 4

// Joint index definitions (for sdhinget, etc.)
#define JOINT_BODY_PLANAR 0
#define JOINT_L_HIP 1
#define JOINT_L_KNEE 2
#define JOINT_R_HIP 3
#define JOINT_R_KNEE 4

/************************************************************************/
/* Single support SDFAST model b1ss */

/* Sizes */
#define SS_NQ 5
#define SS_NU 5
#define SS_NSTATE (SS_NQ+SS_NU)

/* Indices for sdfast state */
#define SS_ANKLE1 0
#define SS_KNEE1 1
#define SS_HIP1 2
#define SS_HIP2 3
#define SS_KNEE2 4
#define SS_ANKLE1D 5
#define SS_KNEE1D 6
#define SS_HIP1D 7
#define SS_HIP2D 8
#define SS_KNEE2D 9

// Body index definitions (for sdpointf, etc.)
#define SS_GND_ID -1
#define SS_CALF1_ID 0
#define SS_THIGH1_ID 1
#define SS_TORSO_ID 2
#define SS_THIGH2_ID 3
#define SS_CALF2_ID 4

/************************************************************************/
/* Double support SDFAST model b1ssgg */

/* Sizes */
#define DS_NQ 5
#define DS_NU 5
#define DS_NSTATE (DS_NQ+DS_NU)

/* Indices for sdfast state */
#define DS_ANKLE1 0
#define DS_KNEE1 1
#define DS_HIP1 2
#define DS_HIP2 3
#define DS_KNEE2 4
#define DS_ANKLE1D 5
#define DS_KNEE1D 6
#define DS_HIP1D 7
#define DS_HIP2D 8
#define DS_KNEE2D 9

// Body index definitions (for sdpointf, etc.)
#define DS_GND_ID -1
#define DS_CALF1_ID 0
#define DS_THIGH1_ID 1
#define DS_TORSO_ID 2
#define DS_THIGH2_ID 3
#define DS_CALF2_ID 4
#define DS_LOOP_ID 5

/************************************************************************/
