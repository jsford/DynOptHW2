/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "main.h"
#include "main2.h"

/*****************************************************************************/
/*****************************************************************************/

extern SIM sim;

int n_parameters;

/*****************************************************************************/
/*****************************************************************************/
// Example of optimizing a value
/*****************************************************************************/
/*****************************************************************************/

/*
double run_sim( SIM *sim )
{
  int i;

  for( i = 0; sim->time < sim->duration; i++ )
    {
      controller( sim );
      save_data( sim );
      if ( sim->status == CRASHED )
	break;
      integrate_one_time_step( sim );
    }

  return get_score( sim );
}
*/

double run_sim( double x[] )
{
  reinit_sim( &sim );
  dvector_to_sim(x, sim.n_parameters, sim.params);

  int i;

  for( i = 0; sim.time < sim.duration; i++ )
    {
      controller( &sim );
      save_data( &sim );
      if ( sim.status == CRASHED ) { break; }
      integrate_one_time_step( &sim );
    }

  return get_score( &sim );
}
/*****************************************************************************/

main( int argc, char **argv )
{
  int i;
  PARAMETER *params;
  int n_parameters;
  double score, new_score;

  init_default_parameters( &sim );
  sim.rand_scale = 0;
  sim.controller_print = 1;

  /* Parameter file argument? */
  if ( argc > 1 )
    {
      params = read_parameter_file( argv[1] );
      n_parameters = process_parameters( params, &sim, 1 );
      if ( n_parameters > MAX_N_PARAMETERS )
	{
	  fprintf( stderr, "Too many parameters %d > %d\n",
		   n_parameters, MAX_N_PARAMETERS );
	  exit( -1 );
	}
    }

  sim.duration = 20;
  init_sim( &sim );
  init_data( &sim );

  sim.controller_print = 0;
  sim.params = params;

  // Optimization works on an array of doubles, not a SIM struct
  double dvec_params[sim.n_parameters];
  // Extract the initial params to the double array dvec_params
  parameters_to_dvector( sim.params, dvec_params );


  // Run CMA-ES here

  dvector_to_sim(dvec_params, sim.n_parameters, sim.params);

  // Save the simulation results to a file that looks like d02015 or something.
  write_the_mrdplot_file( &sim );
}

/*****************************************************************************/
