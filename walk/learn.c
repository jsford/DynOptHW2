/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "main.h"
#include "main2.h"

#include "cmaes/src/cmaes_interface.h"

/*****************************************************************************/

extern SIM sim;

/*****************************************************************************/

double run_sim( double x[], int dim)
{
  assert(dim == sim.n_parameters);
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

double run_cmaes( double initial_guess[] ) {
    cmaes_t evo;
    double *arFunvals, *const*pop, *xfinal;
    int i;

    arFunvals = cmaes_init(&evo, 0, initial_guess, NULL, 0, 0, "cmaes/cmaes_initials.par");
    printf("%s\n", cmaes_SayHello(&evo));
    cmaes_ReadSignals(&evo, "cmaes/cmaes_signals.par");

    while(!cmaes_TestForTermination(&evo)) {
        
        pop = cmaes_SamplePopulation(&evo);

        for(i = 0; i < cmaes_Get(&evo, "lambda"); ++i) {
            arFunvals[i] = run_sim(pop[i], (int) cmaes_Get(&evo, "dim"));
        } 

        cmaes_UpdateDistribution(&evo, arFunvals);

        cmaes_ReadSignals(&evo, "cmaes/cmaes_signals.par");
        fflush(stdout);
    }
    printf("Stop:\n%s\n", cmaes_TestForTermination(&evo));
    cmaes_WriteToFile(&evo, "all", "allcmaes.dat");

    xfinal = cmaes_GetNew(&evo, "xmean");
    cmaes_exit(&evo);

    free(xfinal);

    return 0;
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

  double dvec_params[sim.n_parameters];
  parameters_to_dvector(sim.params, dvec_params);

  // Run CMA-ES here
  run_cmaes(dvec_params);

  // Save the simulation results to a file that looks like d02015 or something.
  write_the_mrdplot_file( &sim );
}

/*****************************************************************************/
