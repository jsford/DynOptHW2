/* --------------------------------------------------------- */
/* --------------- A Very Short Example -------------------- */
/* --------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h> /* free() */
#include <stddef.h> /* NULL */
#include "cmaes/src/cmaes_interface.h"
//headers for simulation
#include <math.h>
#include "main.h"
#include "main2.h"


extern SIM sim;
double opt_params[21];
/*Run a simulation/Evaluate the objective function*/
double run_sim_nosave( SIM *sim, double *par )
{

  int i;

  reinit_sim(sim);
  //Assign the optimization parameters to the simulation.
  sim->swing_time=par[0];
  sim->thrust1=par[1];
  sim->swing_hip_target  =par[2];
  sim->swing_hv1   =par[3];
  sim->swing_knee1  =par[4];
  sim->swing_kv1 =par[5];
  sim->swing_ka1 =par[6];
  sim->swing_knee_target =par[7]; 
  sim->swing_kv2  =par[8];
  sim->swing_ka2 =par[9];
  sim->stance_hip_target =par[10]; 
  sim->stance_hv1  =par[11];
  sim->stance_ha1 =par[12];
  sim->pitch_d =par[13];
  sim->stance_kv1 =par[14];
  sim->stance_ka1 =par[15];
  sim->stance_knee_target =par[16];  
  sim->stance_kv2 =par[17];
  sim->stance_ka2 =par[18];
  sim->stance_ankle_torque =par[19];
  sim->swing_ha1=par[20];

  for( i = 0; sim->time < sim->duration; i++ )
    {
      controller( sim );
      if ( sim->status == CRASHED )
  break;
      integrate_one_time_step( sim );
    }

  // write_the_mrdplot_file( sim );
  return get_score( sim );
}


/*Run a simulation/Evaluate the objective function*/
double run_sim( SIM *sim, double* par)
{

  sim->swing_time=par[0];
  sim->thrust1=par[1];
  sim->swing_hip_target  =par[2];
  sim->swing_hv1   =par[3];
  sim->swing_knee1  =par[4];
  sim->swing_kv1 =par[5];
  sim->swing_ka1 =par[6];
  sim->swing_knee_target =par[7]; 
  sim->swing_kv2  =par[8];
  sim->swing_ka2 =par[9];
  sim->stance_hip_target =par[10]; 
  sim->stance_hv1  =par[11];
  sim->stance_ha1 =par[12];
  sim->pitch_d =par[13];
  sim->stance_kv1 =par[14];
  sim->stance_ka1 =par[15];
  sim->stance_knee_target =par[16];  
  sim->stance_kv2 =par[17];
  sim->stance_ka2 =par[18];
  sim->stance_ankle_torque =par[19];
  sim->swing_ha1=par[20];

  int i;

  for( i = 0; sim->time < sim->duration; i++ )
    {
      controller( sim );
      save_data( sim );
      if ( sim->status == CRASHED )
  break;
      integrate_one_time_step( sim );
    }

  // write_the_mrdplot_file( sim );
  return get_score( sim );
}


/* the optimization loop */
int main(int argn, char **args) {
  cmaes_t evo; /* an CMA-ES type struct or "object" */
  double *arFunvals, *const *pop, *xfinal;
  int i,k; 
  int j=0;


  //SIM parameters
  double score, new_score;
  init_default_parameters( &sim );
  sim.rand_scale = 0;
  sim.controller_print = 1;
  init_sim( &sim );
  init_data( &sim );
  sim.controller_print = 0;




  /* Initialize everything into the struct evo, 0 means default */
  arFunvals = cmaes_init(&evo, 0, NULL, NULL, 0, 0, "cmaes/cmaes_initials.par");
  printf("%s\n", cmaes_SayHello(&evo));
  cmaes_ReadSignals(&evo, "cmaes/cmaes_signals.par");  /* write header and initial values */

  /* Iterate until stop criterion holds */
  while(!cmaes_TestForTermination(&evo))
    { 
      /* generate lambda new search points, sample population */
      pop = cmaes_SamplePopulation(&evo); /* do not change content of pop */


      /* Here we may resample each solution point pop[i] until it
	 	 becomes feasible. function is_feasible(...) needs to be
	 	 user-defined.
	 	 Assumptions: the feasible domain is convex, the optimum is
	 	 not on (or very close to) the domain boundary, initialX is
	 	 feasible and initialStandardDeviations are sufficiently small
	 	 to prevent quasi-infinite looping. */
      /* for (i = 0; i < cmaes_Get(&evo, "popsize"); ++i)
           while (!is_feasible(pop[i]))
             cmaes_ReSampleSingle(&evo, i);
      */

      /* evaluate the new search points using fitfun */
      for (i = 0; i < cmaes_Get(&evo, "lambda"); ++i) {
    	  arFunvals[i] = run_sim_nosave( &sim, pop[i]);
        // for (k=0;k<21;k++)
        //   printf("%f",pop[i][k]);
      }

      /* update the search distribution used for cmaes_SamplePopulation() */
      cmaes_UpdateDistribution(&evo, arFunvals);  

      /* read instructions for printing output or changing termination conditions */ 
      cmaes_ReadSignals(&evo, "cmaes/cmaes_signals.par");
      fflush(stdout); /* useful in MinGW */
      
    }
  printf("Stop:\n%s\n",  cmaes_TestForTermination(&evo)); /* print termination reason */
  cmaes_WriteToFile(&evo, "all", "allcmaes.dat");         /* write final results */

  /* get best estimator for the optimum, xmean */
  xfinal = cmaes_GetNew(&evo, "xmean"); /* "xbestever" might be used as well */
  cmaes_exit(&evo); /* release memory */ 

  reinit_sim(&sim);
  run_sim(&sim, xfinal);
  write_the_mrdplot_file( &sim );
  /* do something with final solution and finally release memory */
  free(xfinal); 

  return 0;
}

