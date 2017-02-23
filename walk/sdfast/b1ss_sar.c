/*
Generated 21-Sep-2004 10:33:28 by SD/FAST, Kane's formulation
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

/* These routines are passed to b1ssroot. */

b1ssposfunc(vars,param,resid)
    double vars[5],param[1],resid[1];
{
    int i;
    double pos[5],vel[5];

    for (i = 0; i < 5; i++) {
        vel[i] = 0.;
    }
    b1ssang2st(vars,pos);
    b1ssstate(param[0],pos,vel);
    b1ssperr(resid);
}

b1ssvelfunc(vars,param,resid)
    double vars[5],param[6],resid[1];
{

    b1ssstate(param[5],param,vars);
    b1ssverr(resid);
}

b1ssstatfunc(vars,param,resid)
    double vars[5],param[6],resid[5];
{
    double pos[5],qdotdum[5];

    b1ssang2st(vars,pos);
    b1ssstate(param[5],pos,param);
    b1ssuforce(param[5],pos,param);
    b1ssperr(resid);
    b1ssderiv(qdotdum,&resid[0]);
}

b1ssstdyfunc(vars,param,resid)
    double vars[10],param[1],resid[5];
{
    double pos[5],qdotdum[5];

    b1ssang2st(vars,pos);
    b1ssstate(param[0],pos,&vars[5]);
    b1ssuforce(param[0],pos,&vars[5]);
    b1ssperr(resid);
    b1ssverr(&resid[0]);
    b1ssderiv(qdotdum,&resid[0]);
}

/* This routine is passed to the integrator. */

b1ssmotfunc(time,state,dstate,param,status)
    double time,state[10],dstate[10],param[1];
    int *status;
{

    b1ssstate(time,state,&state[5]);
    b1ssuforce(time,state,&state[5]);
    b1ssderiv(dstate,&dstate[5]);
    *status = 0;
}

/* This routine performs assembly analysis. */

b1ssassemble(time,state,lock,tol,maxevals,fcnt,err)
    double time,state[10];
    int lock[5];
    double tol;
    int maxevals,*fcnt,*err;
{
    double perrs[1],param[1];
    int i;

    b1ssgentime(&i);
    if (i != 103328) {
        b1ssseterr(50,42);
    }
    param[0] = time;
    *err = 0;
    *fcnt = 0;
    b1ssposfunc(state,param,perrs);
    *fcnt = *fcnt+1;
}

/* This routine performs initial velocity analysis. */

b1ssinitvel(time,state,lock,tol,maxevals,fcnt,err)
    double time,state[10];
    int lock[5];
    double tol;
    int maxevals,*fcnt,*err;
{
    double verrs[1],param[6];
    int i;

    b1ssgentime(&i);
    if (i != 103328) {
        b1ssseterr(51,42);
    }
    for (i = 0; i < 5; i++) {
        param[i] = state[i];
    }
    param[5] = time;
    *err = 0;
    *fcnt = 0;
    b1ssvelfunc(&state[5],param,verrs);
    *fcnt = *fcnt+1;
}

/* This routine performs static analysis. */

b1ssstatic(time,state,lock,ctol,tol,maxevals,fcnt,err)
    double time,state[10];
    int lock[5];
    double ctol,tol;
    int maxevals,*fcnt,*err;
{
    double resid[5],param[6],jw[25],dw[200],rw[80];
    int iw[40],rooterr,i;

    b1ssgentime(&i);
    if (i != 103328) {
        b1ssseterr(52,42);
    }
    for (i = 0; i < 5; i++) {
        param[i] = state[5+i];
    }
    param[5] = time;
    b1ssroot(b1ssstatfunc,state,param,5,5,5,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    b1ssstatfunc(state,param,resid);
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

b1sssteady(time,state,lock,ctol,tol,maxevals,fcnt,err)
    double time,state[10];
    int lock[10];
    double ctol,tol;
    int maxevals,*fcnt,*err;
{
    double resid[5],param[1];
    double jw[50],dw[450],rw[125];
    int iw[60],rooterr,i;

    b1ssgentime(&i);
    if (i != 103328) {
        b1ssseterr(53,42);
    }
    param[0] = time;
    b1ssroot(b1ssstdyfunc,state,param,5,10,5,lock,
      ctol,tol,maxevals,jw,dw,rw,iw,resid,fcnt,&rooterr);
    b1ssstdyfunc(state,param,resid);
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

b1ssmotion(time,state,dstate,dt,ctol,tol,flag,err)
    double *time,state[10],dstate[10],dt,ctol,tol;
    int *flag,*err;
{
    static double step;
    double work[60],ttime,param[1];
    int vintgerr,which,ferr,i;

    b1ssgentime(&i);
    if (i != 103328) {
        b1ssseterr(54,42);
    }
    param[0] = ctol;
    ttime = *time;
    if (*flag != 0) {
        b1ssmotfunc(ttime,state,dstate,param,&ferr);
        step = dt;
        *flag = 0;
    }
    if (step <= 0.) {
        step = dt;
    }
    b1ssvinteg(b1ssmotfunc,&ttime,state,dstate,param,dt,&step,10,tol,work,&
      vintgerr,&which);
    *time = ttime;
    *err = vintgerr;
}

/* This routine performs state integration with a fixed-step integrator. */

b1ssfmotion(time,state,dstate,dt,ctol,flag,errest,err)
    double *time,state[10],dstate[10],dt,ctol;
    int *flag;
    double *errest;
    int *err;
{
    double work[40],ttime,param[1];
    int ferr,i;

    b1ssgentime(&i);
    if (i != 103328) {
        b1ssseterr(55,42);
    }
    param[0] = ctol;
    *err = 0;
    ttime = *time;
    if (*flag != 0) {
        b1ssmotfunc(ttime,state,dstate,param,&ferr);
        *flag = 0;
    }
    b1ssfinteg(b1ssmotfunc,&ttime,state,dstate,param,dt,10,work,errest,&ferr);
    if (ferr != 0) {
        *err = 1;
    }
    *time = ttime;
}
