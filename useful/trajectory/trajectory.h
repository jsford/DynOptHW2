/*****************************************************************************/
/*****************************************************************************/

/* Trajectory Types */
#define NOT_INCLUDED 0
#define LINEAR_SPLINE 1 
#define CUBIC_SPLINE 3 
#define QUINTIC_SPLINE 5 

/*****************************************************************************/

typedef struct knot3
{
  int type;
  double time;
  double position;
  double velocity;
  double acceleration;
  int id; /* unique ID, so can access specific knots */
  struct knot3 *previous;
  struct knot3 *next;
} KNOT3;

typedef struct dofs
{
  char *name;
  int dof;
  int n_knots;
  KNOT3 *knots;
} DOFS;

/*****************************************************************************/

int init_trajectory( DOFS *traj, int n_dof );
int print_trajectory( DOFS *traj, int n_dof );
int set_first_knot_to( DOFS *current_desireds, int dof, int type, double time, 
		   double position, double velocity, double acceleration );
int clear_knots( DOFS *traj, int dof );
int clear_all_knots( DOFS *traj, int n_dof );
KNOT3 *add_knot_point( DOFS *traj, int dof, int type, double time, 
		double position, double velocity, double acceleration );
KNOT3 *add_knot_point_array( DOFS *traj, int dof, int type, double time, 
		double *values );
KNOT3 *add_knot_point_knot( DOFS *traj, int dof, int type, double *values );
int lookup_spline3( DOFS *traj, int dof, double time, DOFS *current_desireds,
		double *p_position, double *p_velocity, double *p_acceleration );
int lookup_spline3_alt( DOFS *traj, int dof, double time, 
			DOFS *current_desireds,
		double *p_position, double *p_velocity, double *p_acceleration );
int 
advance_all_periodic_knots( DOFS *traj, int n_dofs, double time, double amount );

/*****************************************************************************/
/*****************************************************************************/
