/*
Generated 21-Sep-2004 10:34:49 by SD/FAST, Kane's formulation
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

/* These routines are passed to b1dsroot. */

b1dsposfunc(vars,param,resid)
    double vars[5],param[1],resid[5];
{
    int i;
    double pos[5],vel[5];

    for (i = 0; i < 5; i++) {
        vel[i] = 0.;
    }
    b1dsang2st(vars,pos);
    b1dsstate(param[0],pos,vel);
    b1dsperr(resid);
}

b1dsvelfunc(vars,param,resid)
    double vars[5],param[6],resid[5];
{

    b1dsstate(param[5],param,vars);
    b1dsverr(resid);
}

b1dsstatfunc(vars,param,resid)
    double vars[5],param[6],resid[10];
{
    double pos[5],qdotdum[5];

    b1dsang2st(vars,pos);
    b1dsstate(param[5],pos,param);
    b1dsuforce(param[5],pos,param);
    b1dsperr(resid);
    b1dsderiv(qdotdum,&resid[5]);
}

b1dsstdyfunc(vars,param,resid)
    double vars[10],param[1],resid[15];
{
    double pos[5],qdotdum[5];

    b1dsang2st(vars,pos);
    b1dsstate(param[0],pos,&vars[5]);
    b1dsuforce(param[0],pos,&vars[5]);
    b1dsperr(resid);
    b1dsverr(&resid[5]);
    b1dsderiv(qdotdum,&resid[10]);
}

/* This routine is passed to the integrator. */

b1dsmotfunc(time,state,dstate,param,status)
    double time,state[10],dstate[10],param[1];
    int *status;
{
    double err[5];
    int i;

    b1dsstate(time,state,&state[5]);
    b1dsuforce(time,state,&state[5]);
    b1dsderiv(dstate,&dstate[5]);
    *status = 1;
    b1dsverr(err);
    for (i = 0; i < 5; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    b1dsperr(err);
    for (i = 0; i < 5; i++) {
        if (fabs(err[i]) > param[0]) {
            return;
        }
    }
    *status = 0;
}

/* This routine performs assembly analysis. */

b1dsassemble(time,state,lock,tol,maxevals,fcnt,err)
    double time,state[10];
    int lock[5];
    double tol;
    int maxevals,*fcnt,*err;
{
    double perrs[5],param[1];
    int i;
    double jw[25],dw[200],rw[80];
    int iw[40],rooterr;

    b1dsgentime(&i);
    if (i != 103448) {
        b1dsseterr(50,42);
    }
    param[0] = time;
    b1dsroot(b1dsposfunc,state,param,5,5,0,lock,tol,tol,maxevals,
      jw,dw,rw,iw,perrs,fcnt,&rooterr);
    b1dsposfunc(state,param,perrs);
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

b1dsinitvel(time,state,lock,tol,maxevals,fcnt,err)
    double time,state[10];
    int lock[5];
    double tol;
    int maxevals,*fcnt,*err;
{
    double verrs[5],param[6];
    int i;
    double jw[25],dw[200],rw[80];
    int iw[40],rooterr;

    b1dsgentime(&i);
    if (i != 103448) {
        b1dsseterr(51,42);
    }
    for (i = 0; i < 5; i++) {
        param[i] = state[i];
    }
    param[5] = time;
    b1dsroot(b1dsvelfunc,&state[5],param,5,5,0,lock,tol,tol,maxevals,
      jw,dw,rw,iw,verrs,fcnt,&rooterr);
    b1dsvelfunc(&state[5],param,verrs);
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

b1dsstatic(time,state,lock,ctol,tol,maxevals,fcnt,err)
    double time,state[10];
    int lock[5];
    double ctol,tol;
    int maxevals,*fcnt,*err;
{
    double resid[10],param[6],jw[50],dw[450],rw[115];
    int iw[60],rooterr,i;

    b1dsgentime(&i);
    if (i != 103448) {
        b1dsseterr(52,42);
    }
    for (i = 0; i < 5; i++) {
        param[i] = state[5+i];
    }
    param[5] = time;
    b1dsroot(b1dsstatfunc,state,param,10,5,5,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    b1dsstatfunc(state,param,resid);
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

b1dssteady(time,state,lock,ctol,tol,maxevals,fcnt,err)
    double time,state[10];
    int lock[10];
    double ctol,tol;
    int maxevals,*fcnt,*err;
{
    double resid[15],param[1];
    double jw[150],dw[1250],rw[195];
    int iw[100],rooterr,i;

    b1dsgentime(&i);
    if (i != 103448) {
        b1dsseterr(53,42);
    }
    param[0] = time;
    b1dsroot(b1dsstdyfunc,state,param,15,10,5,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    b1dsstdyfunc(state,param,resid);
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

b1dsmotion(time,state,dstate,dt,ctol,tol,flag,err)
    double *time,state[10],dstate[10],dt,ctol,tol;
    int *flag,*err;
{
    static double step;
    double work[60],ttime,param[1];
    int vintgerr,which,ferr,i;

    b1dsgentime(&i);
    if (i != 103448) {
        b1dsseterr(54,42);
    }
    param[0] = ctol;
    ttime = *time;
    if (*flag != 0) {
        b1dsmotfunc(ttime,state,dstate,param,&ferr);
        step = dt;
        *flag = 0;
    }
    if (step <= 0.) {
        step = dt;
    }
    b1dsvinteg(b1dsmotfunc,&ttime,state,dstate,param,dt,&step,10,tol,work,&
      vintgerr,&which);
    *time = ttime;
    *err = vintgerr;
}

/* This routine performs state integration with a fixed-step integrator. */

b1dsfmotion(time,state,dstate,dt,ctol,flag,errest,err)
    double *time,state[10],dstate[10],dt,ctol;
    int *flag;
    double *errest;
    int *err;
{
    double work[40],ttime,param[1];
    int ferr,i;

    b1dsgentime(&i);
    if (i != 103448) {
        b1dsseterr(55,42);
    }
    param[0] = ctol;
    *err = 0;
    ttime = *time;
    if (*flag != 0) {
        b1dsmotfunc(ttime,state,dstate,param,&ferr);
        *flag = 0;
    }
    b1dsfinteg(b1dsmotfunc,&ttime,state,dstate,param,dt,10,work,errest,&ferr);
    if (ferr != 0) {
        *err = 1;
    }
    *time = ttime;
}
