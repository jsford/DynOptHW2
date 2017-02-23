/*
Generated 21-Sep-2004 10:30:58 by SD/FAST, Kane's formulation
(sdfast B.2.6 #70405) on machine ID 690c54f5
Copyright (c) 1990-1996 Symbolic Dynamics, Inc.
Copyright (c) 1990-1996 Parametric Technology Corp.
RESTRICTED RIGHTS LEGEND: Use, duplication, or disclosure by the U.S.
Government is subject to restrictions as set forth in subparagraph
(c)(1)(ii) of the Rights in Technical Data and Computer Software
clause at DFARS 52.227-7013 and similar clauses in the FAR and NASA
FAR Supplement.  Symbolic Dynamics, Inc., Mountain View, CA 94041
*/
#include <math.h>

/* These routines are passed to b1groot. */

b1gposfunc(vars,param,resid)
    double vars[7],param[1],resid[1];
{
    int i;
    double pos[7],vel[7];

    for (i = 0; i < 7; i++) {
        vel[i] = 0.;
    }
    b1gang2st(vars,pos);
    b1gstate(param[0],pos,vel);
    b1gperr(resid);
}

b1gvelfunc(vars,param,resid)
    double vars[7],param[8],resid[1];
{

    b1gstate(param[7],param,vars);
    b1gverr(resid);
}

b1gstatfunc(vars,param,resid)
    double vars[7],param[8],resid[7];
{
    double pos[7],qdotdum[7];

    b1gang2st(vars,pos);
    b1gstate(param[7],pos,param);
    b1guforce(param[7],pos,param);
    b1gperr(resid);
    b1gderiv(qdotdum,&resid[0]);
}

b1gstdyfunc(vars,param,resid)
    double vars[14],param[1],resid[7];
{
    double pos[7],qdotdum[7];

    b1gang2st(vars,pos);
    b1gstate(param[0],pos,&vars[7]);
    b1guforce(param[0],pos,&vars[7]);
    b1gperr(resid);
    b1gverr(&resid[0]);
    b1gderiv(qdotdum,&resid[0]);
}

/* This routine is passed to the integrator. */

b1gmotfunc(time,state,dstate,param,status)
    double time,state[14],dstate[14],param[1];
    int *status;
{

    b1gstate(time,state,&state[7]);
    b1guforce(time,state,&state[7]);
    b1gderiv(dstate,&dstate[7]);
    *status = 0;
}

/* This routine performs assembly analysis. */

b1gassemble(time,state,lock,tol,maxevals,fcnt,err)
    double time,state[14];
    int lock[7];
    double tol;
    int maxevals,*fcnt,*err;
{
    double perrs[1],param[1];
    int i;

    b1ggentime(&i);
    if (i != 103057) {
        b1gseterr(50,42);
    }
    param[0] = time;
    *err = 0;
    *fcnt = 0;
    b1gposfunc(state,param,perrs);
    *fcnt = *fcnt+1;
}

/* This routine performs initial velocity analysis. */

b1ginitvel(time,state,lock,tol,maxevals,fcnt,err)
    double time,state[14];
    int lock[7];
    double tol;
    int maxevals,*fcnt,*err;
{
    double verrs[1],param[8];
    int i;

    b1ggentime(&i);
    if (i != 103057) {
        b1gseterr(51,42);
    }
    for (i = 0; i < 7; i++) {
        param[i] = state[i];
    }
    param[7] = time;
    *err = 0;
    *fcnt = 0;
    b1gvelfunc(&state[7],param,verrs);
    *fcnt = *fcnt+1;
}

/* This routine performs static analysis. */

b1gstatic(time,state,lock,ctol,tol,maxevals,fcnt,err)
    double time,state[14];
    int lock[7];
    double ctol,tol;
    int maxevals,*fcnt,*err;
{
    double resid[7],param[8],jw[49],dw[392],rw[112];
    int iw[56],rooterr,i;

    b1ggentime(&i);
    if (i != 103057) {
        b1gseterr(52,42);
    }
    for (i = 0; i < 7; i++) {
        param[i] = state[7+i];
    }
    param[7] = time;
    b1groot(b1gstatfunc,state,param,7,7,7,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    b1gstatfunc(state,param,resid);
    *fcnt = *fcnt+1;
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs steady motion analysis. */

b1gsteady(time,state,lock,ctol,tol,maxevals,fcnt,err)
    double time,state[14];
    int lock[14];
    double ctol,tol;
    int maxevals,*fcnt,*err;
{
    double resid[7],param[1];
    double jw[98],dw[882],rw[175];
    int iw[84],rooterr,i;

    b1ggentime(&i);
    if (i != 103057) {
        b1gseterr(53,42);
    }
    param[0] = time;
    b1groot(b1gstdyfunc,state,param,7,14,7,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    b1gstdyfunc(state,param,resid);
    *fcnt = *fcnt+1;
    if (rooterr == 0) {
        *err = 0;
    } else {
        if (*fcnt >= maxevals) {
            *err = 2;
        } else {
            *err = 1;
        }
    }
}

/* This routine performs state integration. */

b1gmotion(time,state,dstate,dt,ctol,tol,flag,err)
    double *time,state[14],dstate[14],dt,ctol,tol;
    int *flag,*err;
{
    static double step;
    double work[84],ttime,param[1];
    int vintgerr,which,ferr,i;

    b1ggentime(&i);
    if (i != 103057) {
        b1gseterr(54,42);
    }
    param[0] = ctol;
    ttime = *time;
    if (*flag != 0) {
        b1gmotfunc(ttime,state,dstate,param,&ferr);
        step = dt;
        *flag = 0;
    }
    if (step <= 0.) {
        step = dt;
    }
    b1gvinteg(b1gmotfunc,&ttime,state,dstate,param,dt,&step,14,tol,work,&
      vintgerr,&which);
    *time = ttime;
    *err = vintgerr;
}

/* This routine performs state integration with a fixed-step integrator. */

b1gfmotion(time,state,dstate,dt,ctol,flag,errest,err)
    double *time,state[14],dstate[14],dt,ctol;
    int *flag;
    double *errest;
    int *err;
{
    double work[56],ttime,param[1];
    int ferr,i;

    b1ggentime(&i);
    if (i != 103057) {
        b1gseterr(55,42);
    }
    param[0] = ctol;
    *err = 0;
    ttime = *time;
    if (*flag != 0) {
        b1gmotfunc(ttime,state,dstate,param,&ferr);
        *flag = 0;
    }
    b1gfinteg(b1gmotfunc,&ttime,state,dstate,param,dt,14,work,errest,&ferr);
    if (ferr != 0) {
        *err = 1;
    }
    *time = ttime;
}
