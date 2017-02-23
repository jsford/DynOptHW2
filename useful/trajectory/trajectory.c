/*****************************************************************************/

#include "stdio.h"
#include "math.h"
#include "malloc.h"
//#include "string.h"
//#ifndef VX
//#include "strings.h"
//#else
//#include "vx_headers.h"
//#endif
#include "trajectory.h"

/*****************************************************************************/

#define TRUE 1
#define FALSE 0

/***********************************************************************/
/***********************************************************************/
/***********************************************************************/

KNOT3 *free_knots = NULL;

/***********************************************************************/
/***********************************************************************/

int
print_trajectory( DOFS *traj, int n_dofs )
{
  int j, k;
  KNOT3 *knot;

  for ( j = 0; j < n_dofs; j++ )
    {
      printf( "%d: %s %d\n", j, traj[j].name, traj[j].dof );
      knot = traj[j].knots;
      for( k = 0; knot != NULL; k++, knot = knot->next )
	{
	  printf( "  %d %d %7.3f %7.3f %7.3f %7.3f   %d\n", k,
		  knot->type, knot->time, knot->position,
		  knot->velocity, knot->acceleration,
		  knot->id );
	}
    }
  return TRUE;
}

/***********************************************************************/
/***********************************************************************/
/***********************************************************************/

static int
linear_spline( double the_time, 
		     KNOT3 *k0, 
		     KNOT3 *k1, 
		     double *pos, 
		     double *vel, 
		     double *acc )
{  
  double t1;

  if ( the_time == k0->time )
    {
      *pos = k0->position;
      *vel = k0->velocity;
      *acc = k0->acceleration;
      return TRUE;
    }
  if ( the_time == k1->time )
    {
      *pos = k1->position;
      *vel = k1->velocity;
      *acc = k1->acceleration;
      return TRUE;
    }
  if( fabs( (k1->time - k0->time) ) < 1e-5 )
    {
      *pos = k1->position;
      *vel = k1->velocity;
      *acc = k1->acceleration;
      return TRUE;
    }

  t1 = (the_time - k0->time)/(k1->time - k0->time); 
  *pos = k0->position + (k1->position - k0->position)*t1;
  *vel = (k1->position - k0->position)/(k1->time - k0->time);
  *acc = 0;
  return TRUE;
}

/***********************************************************************/

static int
cubic_spline( double the_time, 
		     KNOT3 *k0, 
		     KNOT3 *k1, 
		     double *pos, 
		     double *vel, 
		     double *acc )
{  
  double a, b, c, d;
  double t1, t2, t3;
  double nv0, nv1, np0, np1;

  if ( the_time == k0->time )
    {
      *pos = k0->position;
      *vel = k0->velocity;
      *acc = k0->acceleration;
      return TRUE;
    }
  if ( the_time == k1->time )
    {
      *pos = k1->position;
      *vel = k1->velocity;
      *acc = k1->acceleration;
      return TRUE;
    }
  if( fabs( (k1->time - k0->time) ) < 1e-5 )
    {
      *pos = k1->position;
      *vel = k1->velocity;
      *acc = k1->acceleration;
      return TRUE;
    }
  np0 = k0->position;
  np1 = k1->position;
  nv0 = k0->velocity*(k1->time - k0->time);
  nv1 = k1->velocity*(k1->time - k0->time);

  d = np0;
  c = nv0;
  b = 3*(np1-np0) - nv1 - 2*nv0;
  a = nv1 + nv0 - 2*(np1-np0);
  t1 = (the_time - k0->time)/(k1->time - k0->time); 
  t2 = t1*t1;
  t3 = t2*t1;
  *pos = a*t3 + b*t2 + c*t1 + d;
  *vel = (3*a*t2 + 2*b*t1 + c)/(k1->time - k0->time);
  *acc = (6*a*t1 + 2*b)/((k1->time - k0->time)*(k1->time - k0->time));
  return TRUE;
}

/***********************************************************************/

static int
minimum_jerk_spline( double the_time, 
		     KNOT3 *k0, 
		     KNOT3 *k1, 
		     double *pos, 
		     double *vel, 
		     double *acc )
{  
  double a, b, c, d, e, f;
  double tmp1, tmp2, tmp3;
  double t1, t2, t3, t4, t5;
  double nv0, nv1, na0, na1;

  if ( the_time == k0->time )
    {
      *pos = k0->position;
      *vel = k0->velocity;
      *acc = k0->acceleration;
      return TRUE;
    }
  if ( the_time == k1->time )
    {
      *pos = k1->position;
      *vel = k1->velocity;
      *acc = k1->acceleration;
      return TRUE;
    }
  if( fabs( (k1->time - k0->time) ) < 1e-5 )
    {
      *pos = k1->position;
      *vel = k1->velocity;
      *acc = k1->acceleration;
      return TRUE;
    }
  nv0 = k0->velocity*(k1->time - k0->time);
  nv1 = k1->velocity*(k1->time - k0->time);
  na0 = k0->acceleration*(k1->time - k0->time)*(k1->time - k0->time);
  na1 = k1->acceleration*(k1->time - k0->time)*(k1->time - k0->time);

  f = k0->position;
  e = nv0;
  d = na0/2;
  tmp1 = na0/2 + nv0 + k0->position - k1->position;
  tmp2 = na0 + nv0 - nv1;
  tmp3 = na0 - na1;
  a = - ( tmp3 - 6*tmp2 + 12*tmp1 )/2;
  b = - ( 8*a + 4*tmp2 - 12*tmp1 )/4;
  c = - ( a + b + tmp1 );
  t1 = (the_time - k0->time)/(k1->time - k0->time); 
  t2 = t1*t1;
  t3 = t2*t1;
  t4 = t3*t1;
  t5 = t4*t1;
  *pos = a*t5 + b*t4 + c*t3 + d*t2 + e*t1 + f;
  *vel = (5*a*t4 + 4*b*t3 + 3*c*t2 + 2*d*t1 + e)/(k1->time - k0->time);
  *acc = (20*a*t3 + 12*b*t2 + 6*c*t1 + 2*d)/
    ((k1->time - k0->time)*(k1->time - k0->time));
  return TRUE;
}

/*****************************************************************************/
  
int
init_trajectory( DOFS *traj, int n_dofs )
{
  int j;

  for( j = 0; j < n_dofs; j++ )
    {
      traj[j].name = "";
      traj[j].dof = j;
      traj[j].n_knots = 0;
      traj[j].knots = NULL;
    }
 return TRUE;
}

/*****************************************************************************/

int
clear_knots( DOFS *traj, int dof )
{
  KNOT3 *knot;
  KNOT3 *next_knot = NULL;

  knot = traj[dof].knots;
  traj[dof].knots = NULL;
  traj[dof].n_knots = 0;
  for( ; knot != NULL; knot = next_knot )
    {
      next_knot = knot->next;
      knot->next = free_knots;
      knot->previous = NULL;
      free_knots = knot;
    }
  return TRUE;
}

/*****************************************************************************/

int
clear_all_knots( DOFS *traj, int n_dofs )
{

  int j;

  for( j = 0; j < n_dofs; j++ )
    {
      clear_knots( traj, j );
    }
  return TRUE;
}

/*****************************************************************************/

int next_id = 1000;
int n_malloc = 0;

KNOT3 *
add_knot_point( DOFS *traj, int dof, int type, double time, 
		double position, double velocity, double acceleration )
{
  KNOT3 *knot;
  KNOT3 **parent_next = NULL;
  KNOT3 *parent_previous = NULL;

  if ( free_knots != NULL )
    {
      knot = free_knots;
      free_knots = knot->next;
    }
  else
    {
      knot = (KNOT3 *) malloc( sizeof( KNOT3 ) );
      n_malloc++;
    }
  if ( knot == NULL )
    {
      printf( "Failed to allocate knot.\n" );
      return NULL;
    }

  knot->type = type;
  knot->time = time;
  knot->position = position;
  knot->velocity = velocity;
  knot->acceleration = acceleration;
  knot->id = next_id++;
  knot->next = NULL;
  knot->previous = NULL;

  /*
  printf( "Added knot %d %d %d\n", knot->type, knot->id, type );
  */

  traj[dof].n_knots += 1;

  /* Just add knot if none there */
  if ( traj[dof].knots == NULL )
    {
      /* printf( "Adding knot %d to NULL dof %d\n", knot->id, dof ); */
      traj[dof].knots = knot;
      return knot;
    }

  /* Add knot at appropriate time */
  for( parent_next = &(traj[dof].knots); 
       ; parent_previous = *parent_next,
	 parent_next = &((*parent_next)->next) )
    {
      if ( (*parent_next) == NULL )
	break;
      if ( knot->time <= (*parent_next)->time )
	break;
    }
  knot->next = *parent_next;
  knot->previous = parent_previous;
  if ( knot->next )
    knot->next->previous = knot;
  *parent_next = knot;

  return knot;
}

/*****************************************************************************/

KNOT3 *
add_knot_point_array( DOFS *traj, int dof, int type, double time,
		      double *values )
{
  return add_knot_point( traj, dof, type, time,
			 values[0], values[1], values[2] );
}

/*****************************************************************************/

KNOT3 *
add_knot_point_knot( DOFS *traj, int dof, int type, double *values )
{
  return add_knot_point( traj, dof, type, 
			 values[0], values[1], values[2], values[3] );
}

/*****************************************************************************/
/*****************************************************************************/
/*
This version of lookup_spline3 creates a spline from a knot point at
current_desireds to the next knot point in traj, instead of one knot point
in traj to the next knot point in traj.
*/

int
lookup_spline3( DOFS *traj, 
		int dof, 
		double time,
		DOFS *current_desireds,
		double *p_position, 
		double *p_velocity, 
		double *p_acceleration )
{
  int i;
  KNOT3 *knot;
  double my_position, my_velocity, my_acceleration;

  for( knot = traj[dof].knots; knot != NULL; knot = knot->next )
    {
      if ( knot->time > time )
	{
	  /* Now do spline interpolation. */
	  switch( knot->type )
	    {
	    case LINEAR_SPLINE:
	      linear_spline( time,
			    current_desireds[dof].knots,
			    knot, 
			    &my_position, &my_velocity, &my_acceleration );
	      break;
	    case CUBIC_SPLINE:
	      cubic_spline( time,
			    current_desireds[dof].knots,
			    knot, 
			    &my_position, &my_velocity, &my_acceleration );
	      break;
	    case QUINTIC_SPLINE:
	      minimum_jerk_spline( time,
			   current_desireds[dof].knots,
			   knot, 
			   &my_position, &my_velocity, &my_acceleration );
	      break;
	    default:
	      printf( "Can't handle knot\n" );
	      return FALSE;
	    }
	  if ( p_position != NULL )
	    *p_position = my_position;
	  if ( p_velocity != NULL )
	    *p_velocity = my_velocity;
	  if ( p_acceleration != NULL )
	    *p_acceleration = my_acceleration;
	  return TRUE;
	}
    }
  /* Failed to find relevant knot */
  if ( current_desireds[dof].knots != NULL )
    {
      if ( p_position != NULL )
	*p_position = current_desireds[dof].knots->position;
      if ( p_velocity != NULL )
	*p_velocity = current_desireds[dof].knots->velocity;
      if ( p_acceleration != NULL )
	*p_acceleration = current_desireds[dof].knots->acceleration;
      return TRUE;
    }
  return FALSE;
}

/*****************************************************************************/
/*****************************************************************************/
/*
This version of lookup_spline3 creates a spline from the appropriate
knot point in traj to the next knot point in traj.
Current_desireds are used if there are no knot points, or the time
desired is before the first knot or after the last knot.
*/

int
lookup_spline3_alt( DOFS *traj, 
		int dof, 
		double time,
		DOFS *current_desireds,
		double *p_position, 
		double *p_velocity, 
		double *p_acceleration )
{
  int i;
  KNOT3 *knot;
  double my_position, my_velocity, my_acceleration;

  knot = traj[dof].knots;

  if ( knot == NULL || knot->time >= time )
    {
      /* Let the other one handle this */
      return lookup_spline3( traj, dof, time, current_desireds,
			     p_position, p_velocity, p_acceleration );
    }
  for( ; knot != NULL; knot = knot->next )
    {
      if ( knot->next == NULL )
	{
	  /* Let the other one handle this */
	  return lookup_spline3( traj, dof, time, current_desireds,
				 p_position, p_velocity, p_acceleration );
	}
      if ( knot->next->time >= time )
	{
	  /*
	  printf( "%g %g %g %g -> %g %g %g %g\n",
		  knot->time, knot->position, 
		  knot->velocity, knot->acceleration,
		  knot->next->time, knot->next->position, 
		  knot->next->velocity, knot->next->acceleration );
	  */
	  /* spline interpolation. */
	  switch( knot->type )
	    {
	    case LINEAR_SPLINE:
	      linear_spline( time,
			     knot,
			     knot->next, 
			     &my_position, &my_velocity, &my_acceleration );
	      break;
	    case CUBIC_SPLINE:
	      cubic_spline( time,
			    knot,
			    knot->next, 
			    &my_position, &my_velocity, &my_acceleration );
	      break;
	    case QUINTIC_SPLINE:
	      minimum_jerk_spline( time,
				   knot,
				   knot->next, 
				   &my_position, &my_velocity,
				   &my_acceleration );
	      break;
	    default:
	      printf( "Can't handle knot" );
	      return FALSE;
	    }
	  if ( p_position != NULL )
	    *p_position = my_position;
	  if ( p_velocity != NULL )
	    *p_velocity = my_velocity;
	  if ( p_acceleration != NULL )
	    *p_acceleration = my_acceleration;
	  return TRUE;
	}
    }
  /* Let the other one handle this */
  return lookup_spline3( traj, dof, time, current_desireds,
			 p_position, p_velocity, p_acceleration );
}

/*****************************************************************************/

int
delete_this_knot( DOFS *traj, int dof, KNOT3 *knot )
{
  if ( knot->previous != NULL )
    {
      knot->previous->next = knot->next;
      if ( knot->next != NULL )
	knot->next->previous = knot->previous;
      knot->next = free_knots;
      knot->previous = NULL;
      free_knots = knot;
      return TRUE;
    }
  traj[dof].knots = knot->next;
  if ( knot->next != NULL )
    knot->next->previous = NULL;
  knot->next = free_knots;
  knot->previous = NULL;
  free_knots = knot;
  return TRUE;
}

/*****************************************************************************/
/* Advance knots earlier than time by 1.0 */

int
advance_periodic_knots( DOFS *traj, int dof, double time, double amount )
{
  KNOT3 *knot, *next_knot;

  /*
  printf( "%d %g %g\n", dof, time, amount );
  */
  for( knot = traj[dof].knots; knot != NULL; knot = next_knot )
    {
      next_knot = knot->next;
      if ( knot->time < 0 )
	continue;
      if ( knot->time < time )
	{
	  add_knot_point( traj, dof, knot->type, knot->time + amount,
			  knot->position, knot->velocity, knot->acceleration );
	  delete_this_knot( traj, dof, knot );
	  /*
	  printf( "Moving knot %d at %g\n", dof, time );
	  print_trajectory( traj, 4 );
	  printf( "Press return to continue.\n" );
	  getchar();
	  */
	}
      else return TRUE;
    }
  return TRUE;
}

/*****************************************************************************/
/* Advance all knots earlier than time by 1.0 */

int 
advance_all_periodic_knots( DOFS *traj, int n_dofs, double time, double amount )
{
  int j;

  /*
  printf( "%d %g %g\n", n_dofs, time, amount );
  */
  for ( j = 0; j < n_dofs; j++ )
    {
      advance_periodic_knots( traj, j, time, amount );
    }
  return TRUE;
}

/*****************************************************************************/

/* Used for setting current desired position */ 
int
set_first_knot_to( DOFS *current_desireds, int dof, int type, double time, 
		   double position, double velocity, double acceleration )
{
  
  if ( current_desireds[dof].knots == NULL )
    {
      /*
      printf( "Adding knot point: %d %d %g %g %g %g\n", dof, QUINTIC_SPLINE,
	      time, position, velocity, acceleration );
	      */
      add_knot_point( current_desireds, dof, type, time, position,
		    velocity, acceleration );
    }
  else
    {
      /* printf( "Assigning values.\n" ); */
      current_desireds[dof].knots->type = type;
      current_desireds[dof].knots->time = time;
      current_desireds[dof].knots->position = position;
      current_desireds[dof].knots->velocity = velocity;
      current_desireds[dof].knots->acceleration = acceleration;
    }
  return TRUE;
}

/*****************************************************************************/

int read_knot( DOFS *traj, int dof, FILE *stream )
{
  int type;
  double time, position, velocity, acceleration;
  
  if ( fscanf( stream, "%d", &type ) < 1 )
    return FALSE;
  switch( type )
    {
    case LINEAR_SPLINE:
      fscanf( stream, "%f%f", &time, &position );
      add_knot_point( traj, dof, type, time, position, 0.0, 0.0 );
      break;
    case CUBIC_SPLINE:
      fscanf( stream, "%f%f%f", &time, &position, &velocity );
      add_knot_point( traj, dof, type, time, position, velocity, 0.0 );
      break;
    case QUINTIC_SPLINE:
      fscanf( stream, "%f%f%f%f", &time, &position, &velocity, &acceleration );
      add_knot_point( traj, dof, type,
		      time, position, velocity, acceleration );
      break;
    default:
      printf( "Couldn't recognize spline type %d\n", type );
      return FALSE;
    }
  return TRUE;
}

/*****************************************************************************/

int read_knots( DOFS *traj, char *filename )
{
  FILE *stream;
  int dof;
  int n_knots;
  int i;
  KNOT3 *knot;

  stream = fopen( filename, "r" );
  printf( "Reading knot file %s\n", filename );
  if ( stream == NULL )
    {
      printf( "Couldn't open knot file %s\n", filename );
      return FALSE;
    }
  for( ; ; )
    {
      /* Any more knots to read? */
      if ( fscanf( stream, "%d", &dof ) < 1 )
	{
	  fclose( stream );
	  return TRUE;
	}
      if ( fscanf( stream, "%d", &n_knots ) < 1 )
	{
	  printf( "Format error 1 in knot file.\n" );
	  fclose ( stream );
	  return FALSE;
	}
      /*
      printf( "DOF %d; n_knots: %d\n", dof, n_knots );
      */
      for ( i = 0; i < n_knots; i++ )
	{
	  if ( !read_knot( traj, dof, stream ) )
	    {
	      fclose( stream );
	      return FALSE;
	    }
	}
    }
}

/*****************************************************************************/
/*****************************************************************************/
