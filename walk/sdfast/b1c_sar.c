/*
Generated 21-Sep-2004 10:32:04 by SD/FAST, Kane's formulation
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

/* These routines are passed to b1croot. */

b1cposfunc(vars,param,resid)
    double vars[7],param[1],resid[4];
{
    int i;
    double pos[7],vel[7];

    for (i = 0; i < 7; i++) {
        vel[i] = 0.;
    }
    b1cang2st(vars,pos);
    b1cstate(param[0],pos,vel);
    b1cperr(resid);
}

b1cvelfunc(vars,param,resid)
    double vars[7],param[8],resid[4];
{

    b1cstate(param[7],param,vars);
    b1cverr(resid);
}

b1cstatfunc(vars,param,resid)
    double vars[7],param[8],resid[11];
{
    double pos[7],qdotdum[7];

    b1cang2st(vars,pos);
    b1cstate(param[7],pos,param);
    b1cuforce(param[7],pos,param);
    b1cperr(resid);
    b1cderiv(qdotdum,&resid[4]);
}

b1cstdyfunc(vars,param,resid)
    double vars[14],param[1],resid[15];
{
    double pos[7],qdotdum[7];

    b1cang2st(vars,pos);
    b1cstate(param[0],pos,&vars[7]);
    b1cuforce(param[0],pos,&vars[7]);
    b1cperr(resid);
    b1cverr(&resid[4]);
    b1cderiv(qdotdum,&resid[8]);
}

/* This routine is passed to the integrator. */

b1cmotfunc(time,state,dstate,param,status)
    double time,state[14],dstate[14],param[1];
    int *status;
{
    double err[4];
    int i;

    b1cstate(time,state,&state[7]);
    b1cuforce(time,state,&state[7]);
    b1cderiv(dstate,&dstate[7]);
    *status = 1;
    b1cverr(err);
    for (i = 0; i < 4; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    b1cperr(err);
    for (i = 0; i < 4; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    *status = 0;
}

/* This routine performs assembly analysis. */

b1cassemble(time,state,lock,tol,maxevals,fcnt,err)
    double time,state[14];
    int lock[7];
    double tol;
    int maxevals,*fcnt,*err;
{
    double perrs[4],param[1];
    int i;
    double jw[28],dw[242],rw[91];
    int iw[44],rooterr;

    b1cgentime(&i);
    if (i != 103203) {
        b1cseterr(50,42);
    }
    param[0] = time;
    b1croot(b1cposfunc,state,param,4,7,0,lock,tol,tol,maxevals,
      jw,dw,rw,iw,perrs,fcnt,&rooterr);
    b1cposfunc(state,param,perrs);
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

/* This routine performs initial velocity analysis. */

b1cinitvel(time,state,lock,tol,maxevals,fcnt,err)
    double time,state[14];
    int lock[7];
    double tol;
    int maxevals,*fcnt,*err;
{
    double verrs[4],param[8];
    int i;
    double jw[28],dw[242],rw[91];
    int iw[44],rooterr;

    b1cgentime(&i);
    if (i != 103203) {
        b1cseterr(51,42);
    }
    for (i = 0; i < 7; i++) {
        param[i] = state[i];
    }
    param[7] = time;
    b1croot(b1cvelfunc,&state[7],param,4,7,0,lock,tol,tol,maxevals,
      jw,dw,rw,iw,verrs,fcnt,&rooterr);
    b1cvelfunc(&state[7],param,verrs);
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

/* This routine performs static analysis. */

b1cstatic(time,state,lock,ctol,tol,maxevals,fcnt,err)
    double time,state[14];
    int lock[7];
    double ctol,tol;
    int maxevals,*fcnt,*err;
{
    double resid[11],param[8],jw[77],dw[648],rw[140];
    int iw[72],rooterr,i;

    b1cgentime(&i);
    if (i != 103203) {
        b1cseterr(52,42);
    }
    for (i = 0; i < 7; i++) {
        param[i] = state[7+i];
    }
    param[7] = time;
    b1croot(b1cstatfunc,state,param,11,7,7,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    b1cstatfunc(state,param,resid);
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

b1csteady(time,state,lock,ctol,tol,maxevals,fcnt,err)
    double time,state[14];
    int lock[14];
    double ctol,tol;
    int maxevals,*fcnt,*err;
{
    double resid[15],param[1];
    double jw[210],dw[1682],rw[231];
    int iw[116],rooterr,i;

    b1cgentime(&i);
    if (i != 103203) {
        b1cseterr(53,42);
    }
    param[0] = time;
    b1croot(b1cstdyfunc,state,param,15,14,7,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    b1cstdyfunc(state,param,resid);
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

b1cmotion(time,state,dstate,dt,ctol,tol,flag,err)
    double *time,state[14],dstate[14],dt,ctol,tol;
    int *flag,*err;
{
    static double step;
    double work[84],ttime,param[1];
    int vintgerr,which,ferr,i;

    b1cgentime(&i);
    if (i != 103203) {
        b1cseterr(54,42);
    }
    param[0] = ctol;
    ttime = *time;
    if (*flag != 0) {
        b1cmotfunc(ttime,state,dstate,param,&ferr);
        step = dt;
        *flag = 0;
    }
    if (step <= 0.) {
        step = dt;
    }
    b1cvinteg(b1cmotfunc,&ttime,state,dstate,param,dt,&step,14,tol,work,&
      vintgerr,&which);
    *time = ttime;
    *err = vintgerr;
}

/* This routine performs state integration with a fixed-step integrator. */

b1cfmotion(time,state,dstate,dt,ctol,flag,errest,err)
    double *time,state[14],dstate[14],dt,ctol;
    int *flag;
    double *errest;
    int *err;
{
    double work[56],ttime,param[1];
    int ferr,i;

    b1cgentime(&i);
    if (i != 103203) {
        b1cseterr(55,42);
    }
    param[0] = ctol;
    *err = 0;
    ttime = *time;
    if (*flag != 0) {
        b1cmotfunc(ttime,state,dstate,param,&ferr);
        *flag = 0;
    }
    b1cfinteg(b1cmotfunc,&ttime,state,dstate,param,dt,14,work,errest,&ferr);
    if (ferr != 0) {
        *err = 1;
    }
    *time = ttime;
}
