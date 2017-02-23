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


ROADMAP (b1ss.sd)

Bodies        Inb
No  Name      body Joint type  Coords q
--- --------- ---- ----------- ----------------
 -1 $ground                                    
  0 l.shin     -1  Pin           0             
  1 l.thigh     0  Pin           1             
  2 pelvis      1  Pin           2             
  3 r.thigh     2  Pin           3             
  4 r.shin      3  Pin           4             

*/
#include <math.h>
#include <stdio.h>

typedef struct {
    int ground_,nbod_,ndof_,ncons_,nloop_,nldof_,nloopc_,nball_,nlball_,npres_,
      nuser_;
    int jtype_[5],inb_[5],outb_[5],njntdof_[5],njntc_[5],njntp_[5],firstq_[5],
      ballq_[5],firstm_[5],firstp_[5];
    int trans_[5];
} b1ssgtopo_t;
#define ground (b1ssgtopo.ground_)
#define nbod (b1ssgtopo.nbod_)
#define ndof (b1ssgtopo.ndof_)
#define ncons (b1ssgtopo.ncons_)
#define nloop (b1ssgtopo.nloop_)
#define nldof (b1ssgtopo.nldof_)
#define nloopc (b1ssgtopo.nloopc_)
#define nball (b1ssgtopo.nball_)
#define nlball (b1ssgtopo.nlball_)
#define npres (b1ssgtopo.npres_)
#define nuser (b1ssgtopo.nuser_)
#define jtype (b1ssgtopo.jtype_)
#define inb (b1ssgtopo.inb_)
#define outb (b1ssgtopo.outb_)
#define njntdof (b1ssgtopo.njntdof_)
#define njntc (b1ssgtopo.njntc_)
#define njntp (b1ssgtopo.njntp_)
#define firstq (b1ssgtopo.firstq_)
#define ballq (b1ssgtopo.ballq_)
#define firstm (b1ssgtopo.firstm_)
#define firstp (b1ssgtopo.firstp_)
#define trans (b1ssgtopo.trans_)

typedef struct {
    double grav_[3],mk_[5],ik_[5][3][3],pin_[5][3];
    double rk_[5][3],ri_[5][3],pres_[5],stabvel_,stabpos_;
    int mfrcflg_,roustate_,vpkflg_,inerflg_,mmflg_,mmlduflg_,wwflg_,ltauflg_,
      fs0flg_,ii_,mmap_[5];
    int gravq_[3],mkq_[5],ikq_[5][3][3],pinq_[5][3],rkq_[5][3],riq_[5][3],presq_
      [5],stabvelq_,stabposq_;
    double mtot_,psmkg_,rhead_[5][3],rcom_[5][3],mkrcomt_[5][3][3],psikg_[3][3],
      psrcomg_[3],psrkg_[3],psrig_[3],psmk_[5],psik_[5][3][3],psrcom_[5][3],
      psrk_[5][3],psri_[5][3];
} b1ssginput_t;
#define grav (b1ssginput.grav_)
#define mk (b1ssginput.mk_)
#define ik (b1ssginput.ik_)
#define pin (b1ssginput.pin_)
#define rk (b1ssginput.rk_)
#define ri (b1ssginput.ri_)
#define pres (b1ssginput.pres_)
#define stabvel (b1ssginput.stabvel_)
#define stabpos (b1ssginput.stabpos_)
#define rhead (b1ssginput.rhead_)
#define rcom (b1ssginput.rcom_)
#define psrcomg (b1ssginput.psrcomg_)
#define psrcom (b1ssginput.psrcom_)
#define mkrcomt (b1ssginput.mkrcomt_)
#define psmk (b1ssginput.psmk_)
#define psik (b1ssginput.psik_)
#define psrk (b1ssginput.psrk_)
#define psri (b1ssginput.psri_)
#define psmkg (b1ssginput.psmkg_)
#define psikg (b1ssginput.psikg_)
#define psrkg (b1ssginput.psrkg_)
#define psrig (b1ssginput.psrig_)
#define mtot (b1ssginput.mtot_)
#define mfrcflg (b1ssginput.mfrcflg_)
#define roustate (b1ssginput.roustate_)
#define vpkflg (b1ssginput.vpkflg_)
#define inerflg (b1ssginput.inerflg_)
#define mmflg (b1ssginput.mmflg_)
#define mmlduflg (b1ssginput.mmlduflg_)
#define wwflg (b1ssginput.wwflg_)
#define ltauflg (b1ssginput.ltauflg_)
#define fs0flg (b1ssginput.fs0flg_)
#define ii (b1ssginput.ii_)
#define mmap (b1ssginput.mmap_)
#define gravq (b1ssginput.gravq_)
#define mkq (b1ssginput.mkq_)
#define ikq (b1ssginput.ikq_)
#define pinq (b1ssginput.pinq_)
#define rkq (b1ssginput.rkq_)
#define riq (b1ssginput.riq_)
#define presq (b1ssginput.presq_)
#define stabvelq (b1ssginput.stabvelq_)
#define stabposq (b1ssginput.stabposq_)

typedef struct {
    double curtim_,q_[5],qn_[5],u_[5],cnk_[5][3][3],cnb_[5][3][3];
    double rnk_[5][3],vnk_[5][3],wk_[5][3],rnb_[5][3],vnb_[5][3],wb_[5][3],
      wbrcom_[5][3],com_[3],rnkg_[3];
    double Cik_[5][3][3],rikt_[5][3][3],Iko_[5][3][3],mkrk_[5][3][3],Cib_[5][3][
      3];
    double Wkk_[5][3],Vkk_[5][3],dik_[5][3],rpp_[5][3],rpk_[5][3],rik_[5][3],
      rik2_[5][3];
    double rpri_[5][3],Wik_[5][3],Vik_[5][3],Wirk_[5][3],rkWkk_[5][3],Wkrpk_[5][
      3],VikWkr_[5][3];
    double perr_[1],verr_[1],aerr_[1],mult_[1],ufk_[5][3],utk_[5][3],mfk_[5][3],
      mtk_[5][3];
    double utau_[5],mtau_[5],uacc_[5],uvel_[5],upos_[5];
    double s0_,c0_,s1_,c1_,s2_,c2_,s3_,c3_,s4_,c4_;
} b1ssgstate_t;
#define curtim (b1ssgstate.curtim_)
#define q (b1ssgstate.q_)
#define qn (b1ssgstate.qn_)
#define u (b1ssgstate.u_)
#define cnk (b1ssgstate.cnk_)
#define cnb (b1ssgstate.cnb_)
#define rnkg (b1ssgstate.rnkg_)
#define rnk (b1ssgstate.rnk_)
#define rnb (b1ssgstate.rnb_)
#define vnk (b1ssgstate.vnk_)
#define vnb (b1ssgstate.vnb_)
#define wk (b1ssgstate.wk_)
#define wb (b1ssgstate.wb_)
#define com (b1ssgstate.com_)
#define Cik (b1ssgstate.Cik_)
#define Cib (b1ssgstate.Cib_)
#define rikt (b1ssgstate.rikt_)
#define Iko (b1ssgstate.Iko_)
#define mkrk (b1ssgstate.mkrk_)
#define Wkk (b1ssgstate.Wkk_)
#define Vkk (b1ssgstate.Vkk_)
#define dik (b1ssgstate.dik_)
#define rpp (b1ssgstate.rpp_)
#define rpk (b1ssgstate.rpk_)
#define rik (b1ssgstate.rik_)
#define rik2 (b1ssgstate.rik2_)
#define rpri (b1ssgstate.rpri_)
#define Wik (b1ssgstate.Wik_)
#define Vik (b1ssgstate.Vik_)
#define Wirk (b1ssgstate.Wirk_)
#define rkWkk (b1ssgstate.rkWkk_)
#define Wkrpk (b1ssgstate.Wkrpk_)
#define VikWkr (b1ssgstate.VikWkr_)
#define wbrcom (b1ssgstate.wbrcom_)
#define perr (b1ssgstate.perr_)
#define verr (b1ssgstate.verr_)
#define aerr (b1ssgstate.aerr_)
#define mult (b1ssgstate.mult_)
#define ufk (b1ssgstate.ufk_)
#define utk (b1ssgstate.utk_)
#define utau (b1ssgstate.utau_)
#define mfk (b1ssgstate.mfk_)
#define mtk (b1ssgstate.mtk_)
#define mtau (b1ssgstate.mtau_)
#define uacc (b1ssgstate.uacc_)
#define uvel (b1ssgstate.uvel_)
#define upos (b1ssgstate.upos_)
#define s0 (b1ssgstate.s0_)
#define c0 (b1ssgstate.c0_)
#define s1 (b1ssgstate.s1_)
#define c1 (b1ssgstate.c1_)
#define s2 (b1ssgstate.s2_)
#define c2 (b1ssgstate.c2_)
#define s3 (b1ssgstate.s3_)
#define c3 (b1ssgstate.c3_)
#define s4 (b1ssgstate.s4_)
#define c4 (b1ssgstate.c4_)

typedef struct {
    double fs0_[5],qdot_[5],Otk_[5][3],Atk_[5][3],AiOiWi_[5][3],Fstar_[5][3];
    double Tstar_[5][3],Fstark_[5][3],Tstark_[5][3],IkWk_[5][3],WkIkWk_[5][3],
      gk_[5][3],IkbWk_[5][3],WkIkbWk_[5][3];
    double w0w0_[5],w1w1_[5],w2w2_[5],w0w1_[5],w0w2_[5],w1w2_[5];
    double w00w11_[5],w00w22_[5],w11w22_[5],ww_[1][1],qraux_[1];
    double mm_[5][5],mlo_[5][5],mdi_[5],IkWpk_[5][5][3],works_[5],workss_[5][5];
    double Wpk_[5][5][3],Vpk_[5][5][3],VWri_[5][5][3];
    int wmap_[1],multmap_[1],jpvt_[1],wsiz_,wrank_;
} b1ssglhs_t;
#define qdot (b1ssglhs.qdot_)
#define Otk (b1ssglhs.Otk_)
#define Atk (b1ssglhs.Atk_)
#define AiOiWi (b1ssglhs.AiOiWi_)
#define Fstar (b1ssglhs.Fstar_)
#define Tstar (b1ssglhs.Tstar_)
#define fs0 (b1ssglhs.fs0_)
#define Fstark (b1ssglhs.Fstark_)
#define Tstark (b1ssglhs.Tstark_)
#define IkWk (b1ssglhs.IkWk_)
#define IkbWk (b1ssglhs.IkbWk_)
#define WkIkWk (b1ssglhs.WkIkWk_)
#define WkIkbWk (b1ssglhs.WkIkbWk_)
#define gk (b1ssglhs.gk_)
#define w0w0 (b1ssglhs.w0w0_)
#define w1w1 (b1ssglhs.w1w1_)
#define w2w2 (b1ssglhs.w2w2_)
#define w0w1 (b1ssglhs.w0w1_)
#define w0w2 (b1ssglhs.w0w2_)
#define w1w2 (b1ssglhs.w1w2_)
#define w00w11 (b1ssglhs.w00w11_)
#define w00w22 (b1ssglhs.w00w22_)
#define w11w22 (b1ssglhs.w11w22_)
#define ww (b1ssglhs.ww_)
#define qraux (b1ssglhs.qraux_)
#define mm (b1ssglhs.mm_)
#define mlo (b1ssglhs.mlo_)
#define mdi (b1ssglhs.mdi_)
#define IkWpk (b1ssglhs.IkWpk_)
#define works (b1ssglhs.works_)
#define workss (b1ssglhs.workss_)
#define Wpk (b1ssglhs.Wpk_)
#define Vpk (b1ssglhs.Vpk_)
#define VWri (b1ssglhs.VWri_)
#define wmap (b1ssglhs.wmap_)
#define multmap (b1ssglhs.multmap_)
#define jpvt (b1ssglhs.jpvt_)
#define wsiz (b1ssglhs.wsiz_)
#define wrank (b1ssglhs.wrank_)

typedef struct {
    double fs_[5],udot_[5],tauc_[5],dyad_[5][3][3],fc_[5][3],tc_[5][3];
    double ank_[5][3],onk_[5][3],Onkb_[5][3],AOnkri_[5][3],Ankb_[5][3],AnkAtk_[5
      ][3],anb_[5][3],onb_[5][3],dyrcom_[5][3];
    double ffk_[5][3],ttk_[5][3],fccikt_[5][3],ffkb_[5][3],ttkb_[5][3];
} b1ssgrhs_t;
#define fs (b1ssgrhs.fs_)
#define udot (b1ssgrhs.udot_)
#define ank (b1ssgrhs.ank_)
#define anb (b1ssgrhs.anb_)
#define onk (b1ssgrhs.onk_)
#define onb (b1ssgrhs.onb_)
#define Onkb (b1ssgrhs.Onkb_)
#define AOnkri (b1ssgrhs.AOnkri_)
#define Ankb (b1ssgrhs.Ankb_)
#define AnkAtk (b1ssgrhs.AnkAtk_)
#define dyrcom (b1ssgrhs.dyrcom_)
#define ffk (b1ssgrhs.ffk_)
#define ttk (b1ssgrhs.ttk_)
#define fccikt (b1ssgrhs.fccikt_)
#define ffkb (b1ssgrhs.ffkb_)
#define ttkb (b1ssgrhs.ttkb_)
#define dyad (b1ssgrhs.dyad_)
#define fc (b1ssgrhs.fc_)
#define tc (b1ssgrhs.tc_)
#define tauc (b1ssgrhs.tauc_)

typedef struct {
    double temp_[3000],tmat1_[3][3],tmat2_[3][3],tvec1_[3],tvec2_[3],tvec3_[3],
      tvec4_[3],tvec5_[3];
    double tsc1_,tsc2_,tsc3_;
} b1ssgtemp_t;
#define temp (b1ssgtemp.temp_)
#define tmat1 (b1ssgtemp.tmat1_)
#define tmat2 (b1ssgtemp.tmat2_)
#define tvec1 (b1ssgtemp.tvec1_)
#define tvec2 (b1ssgtemp.tvec2_)
#define tvec3 (b1ssgtemp.tvec3_)
#define tvec4 (b1ssgtemp.tvec4_)
#define tvec5 (b1ssgtemp.tvec5_)
#define tsc1 (b1ssgtemp.tsc1_)
#define tsc2 (b1ssgtemp.tsc2_)
#define tsc3 (b1ssgtemp.tsc3_)

b1ssgtopo_t b1ssgtopo = {
/*  Topological information
*/
    /* ground */ 1,
    /* nbod */ 5,
    /* ndof */ 5,
    /* ncons */ 0,
    /* nloop */ 0,
    /* nldof */ 0,
    /* nloopc */ 0,
    /* nball */ 0,
    /* nlball */ 0,
    /* npres */ 0,
    /* nuser */ 0,
    /* jtype[0] */ 1,
    /* jtype[1] */ 1,
    /* jtype[2] */ 1,
    /* jtype[3] */ 1,
    /* jtype[4] */ 1,
    /* inb[0] */ -1,
    /* inb[1] */ 0,
    /* inb[2] */ 1,
    /* inb[3] */ 2,
    /* inb[4] */ 3,
    /* outb[0] */ 0,
    /* outb[1] */ 1,
    /* outb[2] */ 2,
    /* outb[3] */ 3,
    /* outb[4] */ 4,
    /* njntdof[0] */ 1,
    /* njntdof[1] */ 1,
    /* njntdof[2] */ 1,
    /* njntdof[3] */ 1,
    /* njntdof[4] */ 1,
    /* njntc[0] */ 0,
    /* njntc[1] */ 0,
    /* njntc[2] */ 0,
    /* njntc[3] */ 0,
    /* njntc[4] */ 0,
    /* njntp[0] */ 0,
    /* njntp[1] */ 0,
    /* njntp[2] */ 0,
    /* njntp[3] */ 0,
    /* njntp[4] */ 0,
    /* firstq[0] */ 0,
    /* firstq[1] */ 1,
    /* firstq[2] */ 2,
    /* firstq[3] */ 3,
    /* firstq[4] */ 4,
    /* ballq[0] */ -104,
    /* ballq[1] */ -104,
    /* ballq[2] */ -104,
    /* ballq[3] */ -104,
    /* ballq[4] */ -104,
    /* firstm[0] */ -1,
    /* firstm[1] */ -1,
    /* firstm[2] */ -1,
    /* firstm[3] */ -1,
    /* firstm[4] */ -1,
    /* firstp[0] */ -1,
    /* firstp[1] */ -1,
    /* firstp[2] */ -1,
    /* firstp[3] */ -1,
    /* firstp[4] */ -1,
    /* trans[0] */ 0,
    /* trans[1] */ 0,
    /* trans[2] */ 0,
    /* trans[3] */ 0,
    /* trans[4] */ 0,
};
b1ssginput_t b1ssginput = {
/* Model parameters from the input file */

/* gravity */
    /* grav[0] */ 0.,
    /* grav[1] */ -9.81,
    /* grav[2] */ 0.,

/* mass */
    /* mk[0] */ .1,
    /* mk[1] */ .54,
    /* mk[2] */ 1.54,
    /* mk[3] */ .54,
    /* mk[4] */ .1,

/* inertia */
    /* ik[0][0][0] */ 1.,
    /* ik[0][0][1] */ 0.,
    /* ik[0][0][2] */ 0.,
    /* ik[0][1][0] */ 0.,
    /* ik[0][1][1] */ 1.,
    /* ik[0][1][2] */ 0.,
    /* ik[0][2][0] */ 0.,
    /* ik[0][2][1] */ 0.,
    /* ik[0][2][2] */ .000349461666666667,
    /* ik[1][0][0] */ 1.,
    /* ik[1][0][1] */ 0.,
    /* ik[1][0][2] */ 0.,
    /* ik[1][1][0] */ 0.,
    /* ik[1][1][1] */ 1.,
    /* ik[1][1][2] */ 0.,
    /* ik[1][2][0] */ 0.,
    /* ik[1][2][1] */ 0.,
    /* ik[1][2][2] */ .000688684713466528,
    /* ik[2][0][0] */ 1.,
    /* ik[2][0][1] */ 0.,
    /* ik[2][0][2] */ 0.,
    /* ik[2][1][0] */ 0.,
    /* ik[2][1][1] */ 1.,
    /* ik[2][1][2] */ 0.,
    /* ik[2][2][0] */ 0.,
    /* ik[2][2][1] */ 0.,
    /* ik[2][2][2] */ .000645073978666667,
    /* ik[3][0][0] */ 1.,
    /* ik[3][0][1] */ 0.,
    /* ik[3][0][2] */ 0.,
    /* ik[3][1][0] */ 0.,
    /* ik[3][1][1] */ 1.,
    /* ik[3][1][2] */ 0.,
    /* ik[3][2][0] */ 0.,
    /* ik[3][2][1] */ 0.,
    /* ik[3][2][2] */ .000688684713466528,
    /* ik[4][0][0] */ 1.,
    /* ik[4][0][1] */ 0.,
    /* ik[4][0][2] */ 0.,
    /* ik[4][1][0] */ 0.,
    /* ik[4][1][1] */ 1.,
    /* ik[4][1][2] */ 0.,
    /* ik[4][2][0] */ 0.,
    /* ik[4][2][1] */ 0.,
    /* ik[4][2][2] */ .000349461666666667,

/* tree hinge axis vectors */
    /* pin[0][0] */ 0.,
    /* pin[0][1] */ 0.,
    /* pin[0][2] */ 1.,
    /* pin[1][0] */ 0.,
    /* pin[1][1] */ 0.,
    /* pin[1][2] */ 1.,
    /* pin[2][0] */ 0.,
    /* pin[2][1] */ 0.,
    /* pin[2][2] */ 1.,
    /* pin[3][0] */ 0.,
    /* pin[3][1] */ 0.,
    /* pin[3][2] */ 1.,
    /* pin[4][0] */ 0.,
    /* pin[4][1] */ 0.,
    /* pin[4][2] */ 1.,

/* tree bodytojoint vectors */
    /* rk[0][0] */ 0.,
    /* rk[0][1] */ -.1016,
    /* rk[0][2] */ 0.,
    /* rk[1][0] */ 0.,
    /* rk[1][1] */ -.131,
    /* rk[1][2] */ 0.,
    /* rk[2][0] */ 0.,
    /* rk[2][1] */ 0.,
    /* rk[2][2] */ 0.,
    /* rk[3][0] */ 0.,
    /* rk[3][1] */ .072,
    /* rk[3][2] */ 0.,
    /* rk[4][0] */ 0.,
    /* rk[4][1] */ .1016,
    /* rk[4][2] */ 0.,

/* tree inbtojoint vectors */
    /* ri[0][0] */ 0.,
    /* ri[0][1] */ 0.,
    /* ri[0][2] */ 0.,
    /* ri[1][0] */ 0.,
    /* ri[1][1] */ .1016,
    /* ri[1][2] */ 0.,
    /* ri[2][0] */ 0.,
    /* ri[2][1] */ .072,
    /* ri[2][2] */ 0.,
    /* ri[3][0] */ 0.,
    /* ri[3][1] */ 0.,
    /* ri[3][2] */ 0.,
    /* ri[4][0] */ 0.,
    /* ri[4][1] */ -.131,
    /* ri[4][2] */ 0.,

/* tree prescribed motion */
    /* pres[0] */ 0.,
    /* pres[1] */ 0.,
    /* pres[2] */ 0.,
    /* pres[3] */ 0.,
    /* pres[4] */ 0.,

/* stabilization parameters */
    /* stabvel */ 0.,
    /* stabpos */ 0.,

/* miscellaneous */
    /* mfrcflg */ 0,
    /* roustate */ 0,
    /* vpkflg */ 0,
    /* inerflg */ 0,
    /* mmflg */ 0,
    /* mmlduflg */ 0,
    /* wwflg */ 0,
    /* ltauflg */ 0,
    /* fs0flg */ 0,
    /* ii */ 0,
    /* mmap[0] */ 0,
    /* mmap[1] */ 1,
    /* mmap[2] */ 2,
    /* mmap[3] */ 3,
    /* mmap[4] */ 4,

/* Which parameters were "?" (1) or "<nominal>?" (3) */
    /* gravq[0] */ 0,
    /* gravq[1] */ 0,
    /* gravq[2] */ 0,
    /* mkq[0] */ 3,
    /* mkq[1] */ 3,
    /* mkq[2] */ 3,
    /* mkq[3] */ 3,
    /* mkq[4] */ 3,
    /* ikq[0][0][0] */ 0,
    /* ikq[0][0][1] */ 0,
    /* ikq[0][0][2] */ 0,
    /* ikq[0][1][0] */ 0,
    /* ikq[0][1][1] */ 0,
    /* ikq[0][1][2] */ 0,
    /* ikq[0][2][0] */ 0,
    /* ikq[0][2][1] */ 0,
    /* ikq[0][2][2] */ 3,
    /* ikq[1][0][0] */ 0,
    /* ikq[1][0][1] */ 0,
    /* ikq[1][0][2] */ 0,
    /* ikq[1][1][0] */ 0,
    /* ikq[1][1][1] */ 0,
    /* ikq[1][1][2] */ 0,
    /* ikq[1][2][0] */ 0,
    /* ikq[1][2][1] */ 0,
    /* ikq[1][2][2] */ 3,
    /* ikq[2][0][0] */ 0,
    /* ikq[2][0][1] */ 0,
    /* ikq[2][0][2] */ 0,
    /* ikq[2][1][0] */ 0,
    /* ikq[2][1][1] */ 0,
    /* ikq[2][1][2] */ 0,
    /* ikq[2][2][0] */ 0,
    /* ikq[2][2][1] */ 0,
    /* ikq[2][2][2] */ 3,
    /* ikq[3][0][0] */ 0,
    /* ikq[3][0][1] */ 0,
    /* ikq[3][0][2] */ 0,
    /* ikq[3][1][0] */ 0,
    /* ikq[3][1][1] */ 0,
    /* ikq[3][1][2] */ 0,
    /* ikq[3][2][0] */ 0,
    /* ikq[3][2][1] */ 0,
    /* ikq[3][2][2] */ 3,
    /* ikq[4][0][0] */ 0,
    /* ikq[4][0][1] */ 0,
    /* ikq[4][0][2] */ 0,
    /* ikq[4][1][0] */ 0,
    /* ikq[4][1][1] */ 0,
    /* ikq[4][1][2] */ 0,
    /* ikq[4][2][0] */ 0,
    /* ikq[4][2][1] */ 0,
    /* ikq[4][2][2] */ 3,
    /* pinq[0][0] */ 0,
    /* pinq[0][1] */ 0,
    /* pinq[0][2] */ 0,
    /* pinq[1][0] */ 0,
    /* pinq[1][1] */ 0,
    /* pinq[1][2] */ 0,
    /* pinq[2][0] */ 0,
    /* pinq[2][1] */ 0,
    /* pinq[2][2] */ 0,
    /* pinq[3][0] */ 0,
    /* pinq[3][1] */ 0,
    /* pinq[3][2] */ 0,
    /* pinq[4][0] */ 0,
    /* pinq[4][1] */ 0,
    /* pinq[4][2] */ 0,
    /* rkq[0][0] */ 0,
    /* rkq[0][1] */ 3,
    /* rkq[0][2] */ 0,
    /* rkq[1][0] */ 0,
    /* rkq[1][1] */ 3,
    /* rkq[1][2] */ 0,
    /* rkq[2][0] */ 0,
    /* rkq[2][1] */ 3,
    /* rkq[2][2] */ 0,
    /* rkq[3][0] */ 0,
    /* rkq[3][1] */ 3,
    /* rkq[3][2] */ 0,
    /* rkq[4][0] */ 0,
    /* rkq[4][1] */ 3,
    /* rkq[4][2] */ 0,
    /* riq[0][0] */ 0,
    /* riq[0][1] */ 0,
    /* riq[0][2] */ 0,
    /* riq[1][0] */ 0,
    /* riq[1][1] */ 3,
    /* riq[1][2] */ 0,
    /* riq[2][0] */ 0,
    /* riq[2][1] */ 3,
    /* riq[2][2] */ 0,
    /* riq[3][0] */ 0,
    /* riq[3][1] */ 3,
    /* riq[3][2] */ 0,
    /* riq[4][0] */ 0,
    /* riq[4][1] */ 3,
    /* riq[4][2] */ 0,
    /* presq[0] */ 0,
    /* presq[1] */ 0,
    /* presq[2] */ 0,
    /* presq[3] */ 0,
    /* presq[4] */ 0,
    /* stabvelq */ 3,
    /* stabposq */ 3,

/* End of values from input file */

};
b1ssgstate_t b1ssgstate;
b1ssglhs_t b1ssglhs;
b1ssgrhs_t b1ssgrhs;
b1ssgtemp_t b1ssgtemp;


b1ssinit()
{
/*
Initialization routine


 This routine must be called before the first call to sdstate(), after
 supplying values for any `?' parameters in the input.
*/
    double sumsq,norminv;
    int i,j,k;


/* Check that all `?' parameters have been assigned values */

    for (k = 0; k < 3; k++) {
        if (gravq[k] == 1) {
            b1ssseterr(7,25);
        }
    }
    for (k = 0; k < 5; k++) {
        if (mkq[k] == 1) {
            b1ssseterr(7,26);
        }
        for (i = 0; i < 3; i++) {
            if (rkq[k][i] == 1) {
                b1ssseterr(7,29);
            }
            if (riq[k][i] == 1) {
                b1ssseterr(7,30);
            }
            for (j = 0; j < 3; j++) {
                if (ikq[k][i][j] == 1) {
                    b1ssseterr(7,27);
                }
            }
        }
    }
    for (k = 0; k < 5; k++) {
        for (i = 0; i < 3; i++) {
            if (pinq[k][i] == 1) {
                b1ssseterr(7,28);
            }
        }
    }

/* Normalize pin vectors if necessary */


/* Zero out Vpk and Wpk */

    for (i = 0; i < 5; i++) {
        for (j = i; j <= 4; j++) {
            for (k = 0; k < 3; k++) {
                Vpk[i][j][k] = 0.;
                Wpk[i][j][k] = 0.;
            }
        }
    }

/* Compute pseudobody-related constants */

    rcom[0][0] = 0.;
    rcom[0][1] = 0.;
    rcom[0][2] = 0.;
    rcom[1][0] = 0.;
    rcom[1][1] = 0.;
    rcom[1][2] = 0.;
    rcom[2][0] = 0.;
    rcom[2][1] = 0.;
    rcom[2][2] = 0.;
    rcom[3][0] = 0.;
    rcom[3][1] = 0.;
    rcom[3][2] = 0.;
    rcom[4][0] = 0.;
    rcom[4][1] = 0.;
    rcom[4][2] = 0.;
    dik[1][1] = (ri[1][1]-rk[0][1]);
    dik[2][1] = (ri[2][1]-rk[1][1]);
    dik[3][1] = (ri[3][1]-rk[2][1]);
    dik[4][1] = (ri[4][1]-rk[3][1]);

/* Compute mass properties-related constants */

    mtot = (mk[4]+(mk[3]+(mk[2]+(mk[0]+mk[1]))));
    mkrk[0][0][2] = (mk[0]*rk[0][1]);
    mkrk[0][2][0] = -(mk[0]*rk[0][1]);
    mkrk[1][0][2] = (mk[1]*rk[1][1]);
    mkrk[1][2][0] = -(mk[1]*rk[1][1]);
    mkrk[2][0][2] = (mk[2]*rk[2][1]);
    mkrk[2][2][0] = -(mk[2]*rk[2][1]);
    mkrk[3][0][2] = (mk[3]*rk[3][1]);
    mkrk[3][2][0] = -(mk[3]*rk[3][1]);
    mkrk[4][0][2] = (mk[4]*rk[4][1]);
    mkrk[4][2][0] = -(mk[4]*rk[4][1]);
    Iko[0][0][0] = (1.+(mkrk[0][0][2]*rk[0][1]));
    Iko[0][2][2] = (ik[0][2][2]-(mkrk[0][2][0]*rk[0][1]));
    Iko[1][0][0] = (1.+(mkrk[1][0][2]*rk[1][1]));
    Iko[1][2][2] = (ik[1][2][2]-(mkrk[1][2][0]*rk[1][1]));
    Iko[2][0][0] = (1.+(mkrk[2][0][2]*rk[2][1]));
    Iko[2][2][2] = (ik[2][2][2]-(mkrk[2][2][0]*rk[2][1]));
    Iko[3][0][0] = (1.+(mkrk[3][0][2]*rk[3][1]));
    Iko[3][2][2] = (ik[3][2][2]-(mkrk[3][2][0]*rk[3][1]));
    Iko[4][0][0] = (1.+(mkrk[4][0][2]*rk[4][1]));
    Iko[4][2][2] = (ik[4][2][2]-(mkrk[4][2][0]*rk[4][1]));
    b1ssserialno(&i);
    if (i != 70405) {
        b1ssseterr(7,41);
    }
    roustate = 1;
}

b1ssstate(timein,qin,uin)
    double timein,qin[5],uin[5];
{
/*
Compute kinematic information and store it in sdgstate.

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
    int i,j,qchg,uchg;

    if ((roustate != 1) && (roustate != 2) && (roustate != 3)) {
        b1ssseterr(8,22);
        return;
    }
    if (roustate == 1) {
        for (i = 0; i < 5; i++) {
            if (presq[i] == 1) {
                b1ssseterr(8,31);
            }
        }
    }
/*
See if time or any qs have changed since last call
*/
    if ((roustate != 1) && (timein == curtim)) {
        qchg = 0;
        for (i = 0; i < 5; i++) {
            if (qin[i] != q[i]) {
                qchg = 1;
                break;
            }
        }
    } else {
        qchg = 1;
    }
/*
If time and qs are unchanged, check us
*/
    if (qchg == 0) {
        uchg = 0;
        for (i = 0; i < 5; i++) {
            if (uin[i] != u[i]) {
                uchg = 1;
                break;
            }
        }
    } else {
        uchg = 1;
    }
    curtim = timein;
    roustate = 2;
    if (qchg == 0) {
        goto skipqs;
    }
/*
Position-related variables need to be computed
*/
    vpkflg = 0;
    mmflg = 0;
    mmlduflg = 0;
    wwflg = 0;
    for (i = 0; i < 5; i++) {
        q[i] = qin[i];
    }
/*
Compute sines and cosines of q
*/
    s0 = sin(q[0]);
    c0 = cos(q[0]);
    s1 = sin(q[1]);
    c1 = cos(q[1]);
    s2 = sin(q[2]);
    c2 = cos(q[2]);
    s3 = sin(q[3]);
    c3 = cos(q[3]);
    s4 = sin(q[4]);
    c4 = cos(q[4]);
/*
Compute across-axis direction cosines Cik
*/
/*
Compute across-joint direction cosines Cib
*/
/*
Compute gravity
*/
    gk[1][0] = -(9.81*((s0*c1)+(s1*c0)));
    gk[1][1] = (9.81*((s0*s1)-(c0*c1)));
    gk[2][0] = ((gk[1][0]*c2)+(gk[1][1]*s2));
    gk[2][1] = ((gk[1][1]*c2)-(gk[1][0]*s2));
    gk[3][0] = ((gk[2][0]*c3)+(gk[2][1]*s3));
    gk[3][1] = ((gk[2][1]*c3)-(gk[2][0]*s3));
    gk[4][0] = ((gk[3][0]*c4)+(gk[3][1]*s4));
    gk[4][1] = ((gk[3][1]*c4)-(gk[3][0]*s4));
/*
Compute cnk & cnb (direction cosines in N)
*/
    cnk[1][0][0] = ((c0*c1)-(s0*s1));
    cnk[1][0][1] = -((s0*c1)+(s1*c0));
    cnk[1][1][0] = ((s0*c1)+(s1*c0));
    cnk[1][1][1] = ((c0*c1)-(s0*s1));
    cnk[2][0][0] = ((cnk[1][0][0]*c2)+(cnk[1][0][1]*s2));
    cnk[2][0][1] = ((cnk[1][0][1]*c2)-(cnk[1][0][0]*s2));
    cnk[2][1][0] = ((cnk[1][1][0]*c2)+(cnk[1][1][1]*s2));
    cnk[2][1][1] = ((cnk[1][1][1]*c2)-(cnk[1][1][0]*s2));
    cnk[3][0][0] = ((cnk[2][0][0]*c3)+(cnk[2][0][1]*s3));
    cnk[3][0][1] = ((cnk[2][0][1]*c3)-(cnk[2][0][0]*s3));
    cnk[3][1][0] = ((cnk[2][1][0]*c3)+(cnk[2][1][1]*s3));
    cnk[3][1][1] = ((cnk[2][1][1]*c3)-(cnk[2][1][0]*s3));
    cnk[4][0][0] = ((cnk[3][0][0]*c4)+(cnk[3][0][1]*s4));
    cnk[4][0][1] = ((cnk[3][0][1]*c4)-(cnk[3][0][0]*s4));
    cnk[4][1][0] = ((cnk[3][1][0]*c4)+(cnk[3][1][1]*s4));
    cnk[4][1][1] = ((cnk[3][1][1]*c4)-(cnk[3][1][0]*s4));
    cnb[0][0][0] = c0;
    cnb[0][0][1] = -s0;
    cnb[0][0][2] = 0.;
    cnb[0][1][0] = s0;
    cnb[0][1][1] = c0;
    cnb[0][1][2] = 0.;
    cnb[0][2][0] = 0.;
    cnb[0][2][1] = 0.;
    cnb[0][2][2] = 1.;
    cnb[1][0][0] = cnk[1][0][0];
    cnb[1][0][1] = cnk[1][0][1];
    cnb[1][0][2] = 0.;
    cnb[1][1][0] = cnk[1][1][0];
    cnb[1][1][1] = cnk[1][1][1];
    cnb[1][1][2] = 0.;
    cnb[1][2][0] = 0.;
    cnb[1][2][1] = 0.;
    cnb[1][2][2] = 1.;
    cnb[2][0][0] = cnk[2][0][0];
    cnb[2][0][1] = cnk[2][0][1];
    cnb[2][0][2] = 0.;
    cnb[2][1][0] = cnk[2][1][0];
    cnb[2][1][1] = cnk[2][1][1];
    cnb[2][1][2] = 0.;
    cnb[2][2][0] = 0.;
    cnb[2][2][1] = 0.;
    cnb[2][2][2] = 1.;
    cnb[3][0][0] = cnk[3][0][0];
    cnb[3][0][1] = cnk[3][0][1];
    cnb[3][0][2] = 0.;
    cnb[3][1][0] = cnk[3][1][0];
    cnb[3][1][1] = cnk[3][1][1];
    cnb[3][1][2] = 0.;
    cnb[3][2][0] = 0.;
    cnb[3][2][1] = 0.;
    cnb[3][2][2] = 1.;
    cnb[4][0][0] = cnk[4][0][0];
    cnb[4][0][1] = cnk[4][0][1];
    cnb[4][0][2] = 0.;
    cnb[4][1][0] = cnk[4][1][0];
    cnb[4][1][1] = cnk[4][1][1];
    cnb[4][1][2] = 0.;
    cnb[4][2][0] = 0.;
    cnb[4][2][1] = 0.;
    cnb[4][2][2] = 1.;
/*
Compute q-related auxiliary variables
*/
    rik[1][0] = (ri[1][1]*s1);
    rik[1][1] = ((ri[1][1]*c1)-rk[1][1]);
    rik[2][0] = (ri[2][1]*s2);
    rik[2][1] = ((ri[2][1]*c2)-rk[2][1]);
    rik[3][0] = (ri[3][1]*s3);
    rik[3][1] = ((ri[3][1]*c3)-rk[3][1]);
    rik[4][0] = (ri[4][1]*s4);
    rik[4][1] = ((ri[4][1]*c4)-rk[4][1]);
/*
Compute rnk & rnb (mass center locations in N)
*/
    rnk[0][0] = (rk[0][1]*s0);
    rnk[0][1] = -(rk[0][1]*c0);
    rnk[1][0] = ((rnk[0][0]-(ri[1][1]*s0))-(cnk[1][0][1]*rk[1][1]));
    rnk[1][1] = ((rnk[0][1]+(ri[1][1]*c0))-(cnk[1][1][1]*rk[1][1]));
    rnk[2][0] = ((rnk[1][0]+(cnk[1][0][1]*ri[2][1]))-(cnk[2][0][1]*rk[2][1]));
    rnk[2][1] = ((rnk[1][1]+(cnk[1][1][1]*ri[2][1]))-(cnk[2][1][1]*rk[2][1]));
    rnk[3][0] = ((rnk[2][0]+(cnk[2][0][1]*ri[3][1]))-(cnk[3][0][1]*rk[3][1]));
    rnk[3][1] = ((rnk[2][1]+(cnk[2][1][1]*ri[3][1]))-(cnk[3][1][1]*rk[3][1]));
    rnk[4][0] = ((rnk[3][0]+(cnk[3][0][1]*ri[4][1]))-(cnk[4][0][1]*rk[4][1]));
    rnk[4][1] = ((rnk[3][1]+(cnk[3][1][1]*ri[4][1]))-(cnk[4][1][1]*rk[4][1]));
    rnb[0][0] = rnk[0][0];
    rnb[0][1] = rnk[0][1];
    rnb[0][2] = 0.;
    rnb[1][0] = rnk[1][0];
    rnb[1][1] = rnk[1][1];
    rnb[1][2] = 0.;
    rnb[2][0] = rnk[2][0];
    rnb[2][1] = rnk[2][1];
    rnb[2][2] = 0.;
    rnb[3][0] = rnk[3][0];
    rnb[3][1] = rnk[3][1];
    rnb[3][2] = 0.;
    rnb[4][0] = rnk[4][0];
    rnb[4][1] = rnk[4][1];
    rnb[4][2] = 0.;
/*
Compute com (system mass center location in N)
*/
    com[0] = ((1./mtot)*((mk[4]*rnk[4][0])+((mk[3]*rnk[3][0])+((mk[2]*rnk[2][0])
      +((mk[0]*rnk[0][0])+(mk[1]*rnk[1][0]))))));
    com[1] = ((1./mtot)*((mk[4]*rnk[4][1])+((mk[3]*rnk[3][1])+((mk[2]*rnk[2][1])
      +((mk[0]*rnk[0][1])+(mk[1]*rnk[1][1]))))));
    com[2] = 0.;
    skipqs: ;
    if (uchg == 0) {
        goto skipus;
    }
/*
Velocity-related variables need to be computed
*/
    inerflg = 0;
    for (i = 0; i < 5; i++) {
        u[i] = uin[i];
    }
/*
Compute u-related auxiliary variables
*/
/*
Compute wk & wb (angular velocities)
*/
    wk[1][2] = (u[0]+u[1]);
    wk[2][2] = (u[2]+wk[1][2]);
    wk[3][2] = (u[3]+wk[2][2]);
    wk[4][2] = (u[4]+wk[3][2]);
    wb[0][0] = 0.;
    wb[0][1] = 0.;
    wb[0][2] = u[0];
    wb[1][0] = 0.;
    wb[1][1] = 0.;
    wb[1][2] = wk[1][2];
    wb[2][0] = 0.;
    wb[2][1] = 0.;
    wb[2][2] = wk[2][2];
    wb[3][0] = 0.;
    wb[3][1] = 0.;
    wb[3][2] = wk[3][2];
    wb[4][0] = 0.;
    wb[4][1] = 0.;
    wb[4][2] = wk[4][2];
/*
Compute auxiliary variables involving wk
*/
    Wirk[1][0] = -(ri[1][1]*u[0]);
    Wirk[2][0] = -(ri[2][1]*wk[1][2]);
    Wirk[3][0] = -(ri[3][1]*wk[2][2]);
    Wirk[4][0] = -(ri[4][1]*wk[3][2]);
    Wkrpk[0][0] = (rk[0][1]*u[0]);
    Wkrpk[1][0] = (rk[1][1]*wk[1][2]);
    Wkrpk[2][0] = (rk[2][1]*wk[2][2]);
    Wkrpk[3][0] = (rk[3][1]*wk[3][2]);
    Wkrpk[4][0] = (rk[4][1]*wk[4][2]);
    IkWk[0][2] = (ik[0][2][2]*u[0]);
    IkWk[1][2] = (ik[1][2][2]*wk[1][2]);
    IkWk[2][2] = (ik[2][2][2]*wk[2][2]);
    IkWk[3][2] = (ik[3][2][2]*wk[3][2]);
    IkWk[4][2] = (ik[4][2][2]*wk[4][2]);
/*
Compute temporaries for use in SDRHS
*/
    w2w2[0] = (u[0]*u[0]);
    w2w2[1] = (wk[1][2]*wk[1][2]);
    w2w2[2] = (wk[2][2]*wk[2][2]);
    w2w2[3] = (wk[3][2]*wk[3][2]);
    w2w2[4] = (wk[4][2]*wk[4][2]);
/*
Compute vnk & vnb (mass center linear velocities in N)
*/
    vnk[0][0] = (Wkrpk[0][0]*c0);
    vnk[0][1] = (Wkrpk[0][0]*s0);
    vnk[1][0] = ((cnk[1][0][0]*Wkrpk[1][0])+(vnk[0][0]+(Wirk[1][0]*c0)));
    vnk[1][1] = ((cnk[1][1][0]*Wkrpk[1][0])+(vnk[0][1]+(Wirk[1][0]*s0)));
    vnk[2][0] = ((cnk[2][0][0]*Wkrpk[2][0])+(vnk[1][0]+(cnk[1][0][0]*Wirk[2][0])
      ));
    vnk[2][1] = ((cnk[2][1][0]*Wkrpk[2][0])+(vnk[1][1]+(cnk[1][1][0]*Wirk[2][0])
      ));
    vnk[3][0] = ((cnk[3][0][0]*Wkrpk[3][0])+(vnk[2][0]+(cnk[2][0][0]*Wirk[3][0])
      ));
    vnk[3][1] = ((cnk[3][1][0]*Wkrpk[3][0])+(vnk[2][1]+(cnk[2][1][0]*Wirk[3][0])
      ));
    vnk[4][0] = ((cnk[4][0][0]*Wkrpk[4][0])+(vnk[3][0]+(cnk[3][0][0]*Wirk[4][0])
      ));
    vnk[4][1] = ((cnk[4][1][0]*Wkrpk[4][0])+(vnk[3][1]+(cnk[3][1][0]*Wirk[4][0])
      ));
    vnb[0][0] = vnk[0][0];
    vnb[0][1] = vnk[0][1];
    vnb[0][2] = 0.;
    vnb[1][0] = vnk[1][0];
    vnb[1][1] = vnk[1][1];
    vnb[1][2] = 0.;
    vnb[2][0] = vnk[2][0];
    vnb[2][1] = vnk[2][1];
    vnb[2][2] = 0.;
    vnb[3][0] = vnk[3][0];
    vnb[3][1] = vnk[3][1];
    vnb[3][2] = 0.;
    vnb[4][0] = vnk[4][0];
    vnb[4][1] = vnk[4][1];
    vnb[4][2] = 0.;
/*
Compute qdot (kinematical equations)
*/
    qdot[0] = u[0];
    qdot[1] = u[1];
    qdot[2] = u[2];
    qdot[3] = u[3];
    qdot[4] = u[4];
    skipus: ;
/*
Initialize applied forces and torques to zero
*/
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 3; j++) {
            ufk[i][j] = 0.;
            utk[i][j] = 0.;
        }
    }
    for (i = 0; i < 5; i++) {
        utau[i] = 0.;
    }
    ltauflg = 0;
    fs0flg = 0;
/*
 Used 0.09 seconds CPU time,
 16384 additional bytes of memory.
 Equations contain   80 adds/subtracts/negates
                    125 multiplies
                      2 divides
                    218 assignments
*/
}

b1ssqdot(oqdot)
    double oqdot[5];
{
/*
Return position coordinate derivatives for tree joints.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(63,23);
        return;
    }
    for (i = 0; i <= 4; i++) {
        oqdot[i] = qdot[i];
    }
}

b1ssu2qdot(uin,oqdot)
    double uin[5],oqdot[5];
{
/*
Convert velocities to qdots.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(64,23);
        return;
    }
    for (i = 0; i <= 4; i++) {
        oqdot[i] = uin[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      5 assignments
*/
}

b1sspsstate(lqin)
    double lqin[1];
{

    if (roustate != 2) {
        b1ssseterr(9,23);
        return;
    }
}

b1ssdovpk()
{

    if (vpkflg == 0) {
/*
Compute Wpk (partial angular velocities)
*/
        Wpk[0][0][2] = 1.;
        Wpk[0][1][2] = 1.;
        Wpk[0][2][2] = 1.;
        Wpk[0][3][2] = 1.;
        Wpk[0][4][2] = 1.;
        Wpk[1][1][2] = 1.;
        Wpk[1][2][2] = 1.;
        Wpk[1][3][2] = 1.;
        Wpk[1][4][2] = 1.;
        Wpk[2][2][2] = 1.;
        Wpk[2][3][2] = 1.;
        Wpk[2][4][2] = 1.;
        Wpk[3][3][2] = 1.;
        Wpk[3][4][2] = 1.;
        Wpk[4][4][2] = 1.;
/*
Compute Vpk (partial velocities)
*/
        Vpk[0][0][0] = rk[0][1];
        VWri[0][1][0] = (rk[0][1]-ri[1][1]);
        Vpk[0][1][0] = (rk[1][1]+(VWri[0][1][0]*c1));
        Vpk[0][1][1] = -(VWri[0][1][0]*s1);
        VWri[0][2][0] = (Vpk[0][1][0]-ri[2][1]);
        Vpk[0][2][0] = (rk[2][1]+((Vpk[0][1][1]*s2)+(VWri[0][2][0]*c2)));
        Vpk[0][2][1] = ((Vpk[0][1][1]*c2)-(VWri[0][2][0]*s2));
        VWri[0][3][0] = (Vpk[0][2][0]-ri[3][1]);
        Vpk[0][3][0] = (rk[3][1]+((Vpk[0][2][1]*s3)+(VWri[0][3][0]*c3)));
        Vpk[0][3][1] = ((Vpk[0][2][1]*c3)-(VWri[0][3][0]*s3));
        VWri[0][4][0] = (Vpk[0][3][0]-ri[4][1]);
        Vpk[0][4][0] = (rk[4][1]+((Vpk[0][3][1]*s4)+(VWri[0][4][0]*c4)));
        Vpk[0][4][1] = ((Vpk[0][3][1]*c4)-(VWri[0][4][0]*s4));
        Vpk[1][1][0] = rk[1][1];
        VWri[1][2][0] = (rk[1][1]-ri[2][1]);
        Vpk[1][2][0] = (rk[2][1]+(VWri[1][2][0]*c2));
        Vpk[1][2][1] = -(VWri[1][2][0]*s2);
        VWri[1][3][0] = (Vpk[1][2][0]-ri[3][1]);
        Vpk[1][3][0] = (rk[3][1]+((Vpk[1][2][1]*s3)+(VWri[1][3][0]*c3)));
        Vpk[1][3][1] = ((Vpk[1][2][1]*c3)-(VWri[1][3][0]*s3));
        VWri[1][4][0] = (Vpk[1][3][0]-ri[4][1]);
        Vpk[1][4][0] = (rk[4][1]+((Vpk[1][3][1]*s4)+(VWri[1][4][0]*c4)));
        Vpk[1][4][1] = ((Vpk[1][3][1]*c4)-(VWri[1][4][0]*s4));
        Vpk[2][2][0] = rk[2][1];
        VWri[2][3][0] = (rk[2][1]-ri[3][1]);
        Vpk[2][3][0] = (rk[3][1]+(VWri[2][3][0]*c3));
        Vpk[2][3][1] = -(VWri[2][3][0]*s3);
        VWri[2][4][0] = (Vpk[2][3][0]-ri[4][1]);
        Vpk[2][4][0] = (rk[4][1]+((Vpk[2][3][1]*s4)+(VWri[2][4][0]*c4)));
        Vpk[2][4][1] = ((Vpk[2][3][1]*c4)-(VWri[2][4][0]*s4));
        Vpk[3][3][0] = rk[3][1];
        VWri[3][4][0] = (rk[3][1]-ri[4][1]);
        Vpk[3][4][0] = (rk[4][1]+(VWri[3][4][0]*c4));
        Vpk[3][4][1] = -(VWri[3][4][0]*s4);
        Vpk[4][4][0] = rk[4][1];
        vpkflg = 1;
    }
/*
 Used 0.03 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain   36 adds/subtracts/negates
                     32 multiplies
                      0 divides
                     50 assignments
*/
}

b1ssdoltau()
{

/*
Compute effect of loop hinge torques
*/
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      0 assignments
*/
}

b1ssdoiner()
{

/*
Compute inertial accelerations and related temps
*/
    if (inerflg == 0) {
/*
Compute Otk (inertial angular acceleration)
*/
/*
Compute Atk (inertial linear acceleration)
*/
        Atk[0][1] = (u[0]*Wkrpk[0][0]);
        AiOiWi[1][1] = (Atk[0][1]+(u[0]*Wirk[1][0]));
        Atk[1][0] = (AiOiWi[1][1]*s1);
        Atk[1][1] = ((AiOiWi[1][1]*c1)+(wk[1][2]*Wkrpk[1][0]));
        AiOiWi[2][1] = (Atk[1][1]+(Wirk[2][0]*wk[1][2]));
        Atk[2][0] = ((AiOiWi[2][1]*s2)+(Atk[1][0]*c2));
        Atk[2][1] = ((wk[2][2]*Wkrpk[2][0])+((AiOiWi[2][1]*c2)-(Atk[1][0]*s2)));
        AiOiWi[3][1] = (Atk[2][1]+(Wirk[3][0]*wk[2][2]));
        Atk[3][0] = ((AiOiWi[3][1]*s3)+(Atk[2][0]*c3));
        Atk[3][1] = ((wk[3][2]*Wkrpk[3][0])+((AiOiWi[3][1]*c3)-(Atk[2][0]*s3)));
        AiOiWi[4][1] = (Atk[3][1]+(Wirk[4][0]*wk[3][2]));
        Atk[4][0] = ((AiOiWi[4][1]*s4)+(Atk[3][0]*c4));
        Atk[4][1] = ((wk[4][2]*Wkrpk[4][0])+((AiOiWi[4][1]*c4)-(Atk[3][0]*s4)));
        inerflg = 1;
    }
/*
 Used 0.01 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   14 adds/subtracts/negates
                     23 multiplies
                      0 divides
                     13 assignments
*/
}

b1ssdofs0()
{

/*
Compute effect of all applied loads
*/
    if (fs0flg == 0) {
        b1ssdoltau();
        b1ssdoiner();
/*
Compute Fstar (forces)
*/
        Fstar[0][0] = ((9.81*(mk[0]*s0))-ufk[0][0]);
        Fstar[0][1] = ((mk[0]*(Atk[0][1]+(9.81*c0)))-ufk[0][1]);
        Fstar[1][0] = ((mk[1]*(Atk[1][0]-gk[1][0]))-ufk[1][0]);
        Fstar[1][1] = ((mk[1]*(Atk[1][1]-gk[1][1]))-ufk[1][1]);
        Fstar[2][0] = ((mk[2]*(Atk[2][0]-gk[2][0]))-ufk[2][0]);
        Fstar[2][1] = ((mk[2]*(Atk[2][1]-gk[2][1]))-ufk[2][1]);
        Fstar[3][0] = ((mk[3]*(Atk[3][0]-gk[3][0]))-ufk[3][0]);
        Fstar[3][1] = ((mk[3]*(Atk[3][1]-gk[3][1]))-ufk[3][1]);
        Fstar[4][0] = ((mk[4]*(Atk[4][0]-gk[4][0]))-ufk[4][0]);
        Fstar[4][1] = ((mk[4]*(Atk[4][1]-gk[4][1]))-ufk[4][1]);
/*
Compute Tstar (torques)
*/
/*
Compute fs0 (RHS ignoring constraints)
*/
        b1ssdovpk();
        fs0[0] = (utau[0]-((((Fstar[4][0]*Vpk[0][4][0])+(Fstar[4][1]*
          Vpk[0][4][1]))-utk[4][2])+((((Fstar[3][0]*Vpk[0][3][0])+(Fstar[3][1]*
          Vpk[0][3][1]))-utk[3][2])+((((Fstar[0][0]*rk[0][1])-utk[0][2])+(((
          Fstar[1][0]*Vpk[0][1][0])+(Fstar[1][1]*Vpk[0][1][1]))-utk[1][2]))+(((
          Fstar[2][0]*Vpk[0][2][0])+(Fstar[2][1]*Vpk[0][2][1]))-utk[2][2])))));
        fs0[1] = (utau[1]-((((Fstar[4][0]*Vpk[1][4][0])+(Fstar[4][1]*
          Vpk[1][4][1]))-utk[4][2])+((((Fstar[1][0]*rk[1][1])-utk[1][2])+(((
          Fstar[2][0]*Vpk[1][2][0])+(Fstar[2][1]*Vpk[1][2][1]))-utk[2][2]))+(((
          Fstar[3][0]*Vpk[1][3][0])+(Fstar[3][1]*Vpk[1][3][1]))-utk[3][2]))));
        fs0[2] = (utau[2]-((((Fstar[2][0]*rk[2][1])-utk[2][2])+(((Fstar[3][0]*
          Vpk[2][3][0])+(Fstar[3][1]*Vpk[2][3][1]))-utk[3][2]))+(((Fstar[4][0]*
          Vpk[2][4][0])+(Fstar[4][1]*Vpk[2][4][1]))-utk[4][2])));
        fs0[3] = (utau[3]-(((Fstar[3][0]*rk[3][1])-utk[3][2])+(((Fstar[4][0]*
          Vpk[3][4][0])+(Fstar[4][1]*Vpk[3][4][1]))-utk[4][2])));
        fs0[4] = (utau[4]-((Fstar[4][0]*rk[4][1])-utk[4][2]));
        fs0flg = 1;
    }
/*
 Used 0.02 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain   59 adds/subtracts/negates
                     37 multiplies
                      0 divides
                     15 assignments
*/
}

b1ssdomm(routine)
    int routine;
{
    int i;

    if (mmflg == 0) {
/*
Compute mass matrix (MM)
*/
        b1ssdovpk();
        mm[0][0] = ((ik[4][2][2]+(mk[4]*((Vpk[0][4][0]*Vpk[0][4][0])+(
          Vpk[0][4][1]*Vpk[0][4][1]))))+((ik[3][2][2]+(mk[3]*((Vpk[0][3][0]*
          Vpk[0][3][0])+(Vpk[0][3][1]*Vpk[0][3][1]))))+((ik[2][2][2]+(mk[2]*((
          Vpk[0][2][0]*Vpk[0][2][0])+(Vpk[0][2][1]*Vpk[0][2][1]))))+((
          ik[0][2][2]+(mk[0]*(rk[0][1]*rk[0][1])))+(ik[1][2][2]+(mk[1]*((
          Vpk[0][1][0]*Vpk[0][1][0])+(Vpk[0][1][1]*Vpk[0][1][1]))))))));
        mm[0][1] = ((ik[4][2][2]+(mk[4]*((Vpk[0][4][0]*Vpk[1][4][0])+(
          Vpk[0][4][1]*Vpk[1][4][1]))))+((ik[3][2][2]+(mk[3]*((Vpk[0][3][0]*
          Vpk[1][3][0])+(Vpk[0][3][1]*Vpk[1][3][1]))))+((ik[1][2][2]+(mk[1]*(
          rk[1][1]*Vpk[0][1][0])))+(ik[2][2][2]+(mk[2]*((Vpk[0][2][0]*
          Vpk[1][2][0])+(Vpk[0][2][1]*Vpk[1][2][1])))))));
        mm[0][2] = ((ik[4][2][2]+(mk[4]*((Vpk[0][4][0]*Vpk[2][4][0])+(
          Vpk[0][4][1]*Vpk[2][4][1]))))+((ik[2][2][2]+(mk[2]*(rk[2][1]*
          Vpk[0][2][0])))+(ik[3][2][2]+(mk[3]*((Vpk[0][3][0]*Vpk[2][3][0])+(
          Vpk[0][3][1]*Vpk[2][3][1]))))));
        mm[0][3] = ((ik[3][2][2]+(mk[3]*(rk[3][1]*Vpk[0][3][0])))+(ik[4][2][2]+(
          mk[4]*((Vpk[0][4][0]*Vpk[3][4][0])+(Vpk[0][4][1]*Vpk[3][4][1])))));
        mm[0][4] = (ik[4][2][2]+(mk[4]*(rk[4][1]*Vpk[0][4][0])));
        mm[1][1] = ((ik[4][2][2]+(mk[4]*((Vpk[1][4][0]*Vpk[1][4][0])+(
          Vpk[1][4][1]*Vpk[1][4][1]))))+((ik[3][2][2]+(mk[3]*((Vpk[1][3][0]*
          Vpk[1][3][0])+(Vpk[1][3][1]*Vpk[1][3][1]))))+((ik[1][2][2]+(mk[1]*(
          rk[1][1]*rk[1][1])))+(ik[2][2][2]+(mk[2]*((Vpk[1][2][0]*Vpk[1][2][0])+
          (Vpk[1][2][1]*Vpk[1][2][1])))))));
        mm[1][2] = ((ik[4][2][2]+(mk[4]*((Vpk[1][4][0]*Vpk[2][4][0])+(
          Vpk[1][4][1]*Vpk[2][4][1]))))+((ik[2][2][2]+(mk[2]*(rk[2][1]*
          Vpk[1][2][0])))+(ik[3][2][2]+(mk[3]*((Vpk[1][3][0]*Vpk[2][3][0])+(
          Vpk[1][3][1]*Vpk[2][3][1]))))));
        mm[1][3] = ((ik[3][2][2]+(mk[3]*(rk[3][1]*Vpk[1][3][0])))+(ik[4][2][2]+(
          mk[4]*((Vpk[1][4][0]*Vpk[3][4][0])+(Vpk[1][4][1]*Vpk[3][4][1])))));
        mm[1][4] = (ik[4][2][2]+(mk[4]*(rk[4][1]*Vpk[1][4][0])));
        mm[2][2] = ((ik[4][2][2]+(mk[4]*((Vpk[2][4][0]*Vpk[2][4][0])+(
          Vpk[2][4][1]*Vpk[2][4][1]))))+((ik[2][2][2]+(mk[2]*(rk[2][1]*rk[2][1])
          ))+(ik[3][2][2]+(mk[3]*((Vpk[2][3][0]*Vpk[2][3][0])+(Vpk[2][3][1]*
          Vpk[2][3][1]))))));
        mm[2][3] = ((ik[3][2][2]+(mk[3]*(rk[3][1]*Vpk[2][3][0])))+(ik[4][2][2]+(
          mk[4]*((Vpk[2][4][0]*Vpk[3][4][0])+(Vpk[2][4][1]*Vpk[3][4][1])))));
        mm[2][4] = (ik[4][2][2]+(mk[4]*(rk[4][1]*Vpk[2][4][0])));
        mm[3][3] = ((ik[3][2][2]+(mk[3]*(rk[3][1]*rk[3][1])))+(ik[4][2][2]+(
          mk[4]*((Vpk[3][4][0]*Vpk[3][4][0])+(Vpk[3][4][1]*Vpk[3][4][1])))));
        mm[3][4] = (ik[4][2][2]+(mk[4]*(rk[4][1]*Vpk[3][4][0])));
        mm[4][4] = (ik[4][2][2]+(mk[4]*(rk[4][1]*rk[4][1])));
/*
Check for singular mass matrix
*/
        for (i = 0; i < 5; i++) {
            if (mm[i][i] < 1e-13) {
                b1ssseterr(routine,47);
            }
        }
        mmflg = 1;
    }
/*
 Used 0.02 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   75 adds/subtracts/negates
                     90 multiplies
                      0 divides
                     15 assignments
*/
}

b1ssdommldu(routine)
    int routine;
{
    int i;

    if (mmlduflg == 0) {
        b1ssdomm(routine);
/*
Numerically decompose the mass matrix
*/
        b1ssldudcomp(5,5,mmap,1e-13,workss,works,mm,mlo,mdi);
/*
Check for singular mass matrix
*/
        for (i = 0; i < 5; i++) {
            if (mdi[i] <= 1e-13) {
                b1ssseterr(routine,47);
            }
        }
        mmlduflg = 1;
    }
}

b1sslhs(routine)
    int routine;
{
/* Compute all remaining state- and force-dependent quantities
*/

    roustate = 2;
    b1ssdommldu(routine);
    b1ssdofs0();
}

b1ssmassmat(mmat)
    double mmat[5][5];
{
/* Return the system mass matrix (LHS)
*/
    int i,j;

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(57,23);
        return;
    }
    b1ssdomm(57);
    for (i = 0; i < 5; i++) {
        for (j = i; j <= 4; j++) {
            mmat[i][j] = mm[i][j];
            mmat[j][i] = mm[i][j];
        }
    }
}

b1ssfrcmat(fmat)
    double fmat[5];
{
/* Return the system force matrix (RHS), excluding constraints
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(58,23);
        return;
    }
    b1ssdofs0();
    for (i = 0; i < 5; i++) {
        fmat[i] = fs0[i];
    }
}

b1ssmfrc(imult)
    double imult[1];
{

}

b1ssequivht(tau)
    double tau[5];
{
/* Compute tree hinge torques to match effect of applied loads
*/
    double fstareq[5][3],tstareq[5][3];

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(56,23);
        return;
    }
/*
Compute fstareq (forces)
*/
    fstareq[0][0] = ((9.81*(mk[0]*s0))-ufk[0][0]);
    fstareq[0][1] = ((9.81*(mk[0]*c0))-ufk[0][1]);
    fstareq[1][0] = -(ufk[1][0]+(gk[1][0]*mk[1]));
    fstareq[1][1] = -(ufk[1][1]+(gk[1][1]*mk[1]));
    fstareq[2][0] = -(ufk[2][0]+(gk[2][0]*mk[2]));
    fstareq[2][1] = -(ufk[2][1]+(gk[2][1]*mk[2]));
    fstareq[3][0] = -(ufk[3][0]+(gk[3][0]*mk[3]));
    fstareq[3][1] = -(ufk[3][1]+(gk[3][1]*mk[3]));
    fstareq[4][0] = -(ufk[4][0]+(gk[4][0]*mk[4]));
    fstareq[4][1] = -(ufk[4][1]+(gk[4][1]*mk[4]));
/*
Compute tstareq (torques)
*/
/*
Compute taus (RHS ignoring constraints and inertial forces)
*/
    b1ssdovpk();
    tau[0] = (utau[0]-((((fstareq[4][0]*Vpk[0][4][0])+(fstareq[4][1]*
      Vpk[0][4][1]))-utk[4][2])+((((fstareq[3][0]*Vpk[0][3][0])+(fstareq[3][1]*
      Vpk[0][3][1]))-utk[3][2])+((((fstareq[0][0]*rk[0][1])-utk[0][2])+(((
      fstareq[1][0]*Vpk[0][1][0])+(fstareq[1][1]*Vpk[0][1][1]))-utk[1][2]))+(((
      fstareq[2][0]*Vpk[0][2][0])+(fstareq[2][1]*Vpk[0][2][1]))-utk[2][2])))));
    tau[1] = (utau[1]-((((fstareq[4][0]*Vpk[1][4][0])+(fstareq[4][1]*
      Vpk[1][4][1]))-utk[4][2])+((((fstareq[1][0]*rk[1][1])-utk[1][2])+(((
      fstareq[2][0]*Vpk[1][2][0])+(fstareq[2][1]*Vpk[1][2][1]))-utk[2][2]))+(((
      fstareq[3][0]*Vpk[1][3][0])+(fstareq[3][1]*Vpk[1][3][1]))-utk[3][2]))));
    tau[2] = (utau[2]-((((fstareq[2][0]*rk[2][1])-utk[2][2])+(((fstareq[3][0]*
      Vpk[2][3][0])+(fstareq[3][1]*Vpk[2][3][1]))-utk[3][2]))+(((fstareq[4][0]*
      Vpk[2][4][0])+(fstareq[4][1]*Vpk[2][4][1]))-utk[4][2])));
    tau[3] = (utau[3]-(((fstareq[3][0]*rk[3][1])-utk[3][2])+(((fstareq[4][0]*
      Vpk[3][4][0])+(fstareq[4][1]*Vpk[3][4][1]))-utk[4][2])));
    tau[4] = (utau[4]-((fstareq[4][0]*rk[4][1])-utk[4][2]));
/*
Op counts below do not include called subroutines
*/
/*
 Used 0.02 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain   58 adds/subtracts/negates
                     37 multiplies
                      0 divides
                     15 assignments
*/
}

b1ssfulltrq(udotin,multin,trqout)
    double udotin[5],multin[1],trqout[5];
{
/* Compute hinge torques which would produce indicated udots
*/
    double fstarr[5][3],tstarr[5][3],Otkr[5][3],Atir[5][3],Atkr[5][3];

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(61,23);
        return;
    }
/*
Account for inertial accelerations and supplied udots
*/
    Otkr[1][2] = (udotin[0]+udotin[1]);
    Otkr[2][2] = (Otkr[1][2]+udotin[2]);
    Otkr[3][2] = (Otkr[2][2]+udotin[3]);
    Otkr[4][2] = (Otkr[3][2]+udotin[4]);
    Atkr[0][0] = (rk[0][1]*udotin[0]);
    Atkr[0][1] = (u[0]*Wkrpk[0][0]);
    Atir[1][0] = (Atkr[0][0]-(ri[1][1]*udotin[0]));
    Atir[1][1] = (Atkr[0][1]+(u[0]*Wirk[1][0]));
    Atkr[1][0] = ((Otkr[1][2]*rk[1][1])+((Atir[1][0]*c1)+(Atir[1][1]*s1)));
    Atkr[1][1] = ((wk[1][2]*Wkrpk[1][0])+((Atir[1][1]*c1)-(Atir[1][0]*s1)));
    Atir[2][0] = (Atkr[1][0]-(Otkr[1][2]*ri[2][1]));
    Atir[2][1] = (Atkr[1][1]+(Wirk[2][0]*wk[1][2]));
    Atkr[2][0] = ((Otkr[2][2]*rk[2][1])+((Atir[2][0]*c2)+(Atir[2][1]*s2)));
    Atkr[2][1] = ((wk[2][2]*Wkrpk[2][0])+((Atir[2][1]*c2)-(Atir[2][0]*s2)));
    Atir[3][0] = (Atkr[2][0]-(Otkr[2][2]*ri[3][1]));
    Atir[3][1] = (Atkr[2][1]+(Wirk[3][0]*wk[2][2]));
    Atkr[3][0] = ((Otkr[3][2]*rk[3][1])+((Atir[3][0]*c3)+(Atir[3][1]*s3)));
    Atkr[3][1] = ((wk[3][2]*Wkrpk[3][0])+((Atir[3][1]*c3)-(Atir[3][0]*s3)));
    Atir[4][0] = (Atkr[3][0]-(Otkr[3][2]*ri[4][1]));
    Atir[4][1] = (Atkr[3][1]+(Wirk[4][0]*wk[3][2]));
    Atkr[4][0] = ((Otkr[4][2]*rk[4][1])+((Atir[4][0]*c4)+(Atir[4][1]*s4)));
    Atkr[4][1] = ((wk[4][2]*Wkrpk[4][0])+((Atir[4][1]*c4)-(Atir[4][0]*s4)));
/*
Accumulate all forces and torques
*/
    fstarr[0][0] = (ufk[0][0]-(mk[0]*(Atkr[0][0]+(9.81*s0))));
    fstarr[0][1] = (ufk[0][1]-(mk[0]*(Atkr[0][1]+(9.81*c0))));
    fstarr[1][0] = (ufk[1][0]+(mk[1]*(gk[1][0]-Atkr[1][0])));
    fstarr[1][1] = (ufk[1][1]+(mk[1]*(gk[1][1]-Atkr[1][1])));
    fstarr[2][0] = (ufk[2][0]+(mk[2]*(gk[2][0]-Atkr[2][0])));
    fstarr[2][1] = (ufk[2][1]+(mk[2]*(gk[2][1]-Atkr[2][1])));
    fstarr[3][0] = (ufk[3][0]+(mk[3]*(gk[3][0]-Atkr[3][0])));
    fstarr[3][1] = (ufk[3][1]+(mk[3]*(gk[3][1]-Atkr[3][1])));
    fstarr[4][0] = (ufk[4][0]+(mk[4]*(gk[4][0]-Atkr[4][0])));
    fstarr[4][1] = (ufk[4][1]+(mk[4]*(gk[4][1]-Atkr[4][1])));
    tstarr[0][2] = (utk[0][2]-(ik[0][2][2]*udotin[0]));
    tstarr[1][2] = (utk[1][2]-(ik[1][2][2]*Otkr[1][2]));
    tstarr[2][2] = (utk[2][2]-(ik[2][2][2]*Otkr[2][2]));
    tstarr[3][2] = (utk[3][2]-(ik[3][2][2]*Otkr[3][2]));
    tstarr[4][2] = (utk[4][2]-(ik[4][2][2]*Otkr[4][2]));
/*
Now calculate the torques
*/
    b1ssdovpk();
    trqout[0] = -(utau[0]+((tstarr[4][2]+((fstarr[4][0]*Vpk[0][4][0])+(
      fstarr[4][1]*Vpk[0][4][1])))+((tstarr[3][2]+((fstarr[3][0]*Vpk[0][3][0])+(
      fstarr[3][1]*Vpk[0][3][1])))+((tstarr[2][2]+((fstarr[2][0]*Vpk[0][2][0])+(
      fstarr[2][1]*Vpk[0][2][1])))+((tstarr[0][2]+(fstarr[0][0]*rk[0][1]))+(
      tstarr[1][2]+((fstarr[1][0]*Vpk[0][1][0])+(fstarr[1][1]*Vpk[0][1][1]))))))
      ));
    trqout[1] = -(utau[1]+((tstarr[4][2]+((fstarr[4][0]*Vpk[1][4][0])+(
      fstarr[4][1]*Vpk[1][4][1])))+((tstarr[3][2]+((fstarr[3][0]*Vpk[1][3][0])+(
      fstarr[3][1]*Vpk[1][3][1])))+((tstarr[1][2]+(fstarr[1][0]*rk[1][1]))+(
      tstarr[2][2]+((fstarr[2][0]*Vpk[1][2][0])+(fstarr[2][1]*Vpk[1][2][1]))))))
      );
    trqout[2] = -(utau[2]+((tstarr[4][2]+((fstarr[4][0]*Vpk[2][4][0])+(
      fstarr[4][1]*Vpk[2][4][1])))+((tstarr[2][2]+(fstarr[2][0]*rk[2][1]))+(
      tstarr[3][2]+((fstarr[3][0]*Vpk[2][3][0])+(fstarr[3][1]*Vpk[2][3][1]))))))
      ;
    trqout[3] = -(utau[3]+((tstarr[3][2]+(fstarr[3][0]*rk[3][1]))+(tstarr[4][2]+
      ((fstarr[4][0]*Vpk[3][4][0])+(fstarr[4][1]*Vpk[3][4][1])))));
    trqout[4] = -(utau[4]+(tstarr[4][2]+(fstarr[4][0]*rk[4][1])));
/*
Op counts below do not include called subroutines
*/
/*
 Used 0.04 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   98 adds/subtracts/negates
                     76 multiplies
                      0 divides
                     42 assignments
*/
}

b1sscomptrq(udotin,trqout)
    double udotin[5],trqout[5];
{
/* Compute hinge torques to produce these udots, ignoring constraints
*/
    double multin[1];

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(60,23);
        return;
    }
    b1ssfulltrq(udotin,multin,trqout);
}

b1ssmulttrq(multin,trqout)
    double multin[1],trqout[5];
{
/* Compute hinge trqs which would be produced by these mults.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(65,23);
        return;
    }
    for (i = 0; i < 5; i++) {
        trqout[i] = 0.;
    }
}

b1ssfs0()
{

/*
Compute Fs (ignoring multiplier forces)
*/
    fs[0] = fs0[0];
    fs[1] = fs0[1];
    fs[2] = fs0[2];
    fs[3] = fs0[3];
    fs[4] = fs0[4];
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      5 assignments
*/
}

b1ssfsmult()
{
    int i;

/*
Compute Fs (multiplier-generated forces only)
*/
    for (i = 0; i < 5; i++) {
        fs[i] = 0.;
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      5 assignments
*/
}

b1ssfsfull()
{

/*
Compute Fs (including all forces)
*/
    b1ssfsmult();
    fs[0] = (fs[0]+fs0[0]);
    fs[1] = (fs[1]+fs0[1]);
    fs[2] = (fs[2]+fs0[2]);
    fs[3] = (fs[3]+fs0[3]);
    fs[4] = (fs[4]+fs0[4]);
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    5 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      5 assignments
*/
}

b1ssfsgenmult()
{
    int i;

/*
Compute Fs (generic multiplier-generated forces)
*/
    for (i = 0; i < 5; i++) {
        fs[i] = 0.;
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      5 assignments
*/
}

b1ssfsgenfull()
{

/*
Compute Fs (incl generic mult & other forces)
*/
    b1ssfsgenmult();
    fs[0] = (fs[0]+fs0[0]);
    fs[1] = (fs[1]+fs0[1]);
    fs[2] = (fs[2]+fs0[2]);
    fs[3] = (fs[3]+fs0[3]);
    fs[4] = (fs[4]+fs0[4]);
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    5 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      5 assignments
*/
}

b1ssrhs()
{
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

/*
Compute hinge torques for tree hinges
*/
    tauc[0] = utau[0];
    tauc[1] = utau[1];
    tauc[2] = utau[2];
    tauc[3] = utau[3];
    tauc[4] = utau[4];
    b1ssdoiner();
/*
Compute onk & onb (angular accels in N)
*/
    Onkb[1][2] = (udot[0]+udot[1]);
    Onkb[2][2] = (Onkb[1][2]+udot[2]);
    Onkb[3][2] = (Onkb[2][2]+udot[3]);
    Onkb[4][2] = (Onkb[3][2]+udot[4]);
    onb[0][0] = 0.;
    onb[0][1] = 0.;
    onb[0][2] = udot[0];
    onb[1][0] = 0.;
    onb[1][1] = 0.;
    onb[1][2] = Onkb[1][2];
    onb[2][0] = 0.;
    onb[2][1] = 0.;
    onb[2][2] = Onkb[2][2];
    onb[3][0] = 0.;
    onb[3][1] = 0.;
    onb[3][2] = Onkb[3][2];
    onb[4][0] = 0.;
    onb[4][1] = 0.;
    onb[4][2] = Onkb[4][2];
/*
Compute acceleration dyadics
*/
    dyad[0][0][0] = -w2w2[0];
    dyad[0][0][1] = -udot[0];
    dyad[0][0][2] = 0.;
    dyad[0][1][0] = udot[0];
    dyad[0][1][1] = -w2w2[0];
    dyad[0][1][2] = 0.;
    dyad[0][2][0] = 0.;
    dyad[0][2][1] = 0.;
    dyad[0][2][2] = 0.;
    dyad[1][0][0] = -w2w2[1];
    dyad[1][0][1] = -Onkb[1][2];
    dyad[1][0][2] = 0.;
    dyad[1][1][0] = Onkb[1][2];
    dyad[1][1][1] = -w2w2[1];
    dyad[1][1][2] = 0.;
    dyad[1][2][0] = 0.;
    dyad[1][2][1] = 0.;
    dyad[1][2][2] = 0.;
    dyad[2][0][0] = -w2w2[2];
    dyad[2][0][1] = -Onkb[2][2];
    dyad[2][0][2] = 0.;
    dyad[2][1][0] = Onkb[2][2];
    dyad[2][1][1] = -w2w2[2];
    dyad[2][1][2] = 0.;
    dyad[2][2][0] = 0.;
    dyad[2][2][1] = 0.;
    dyad[2][2][2] = 0.;
    dyad[3][0][0] = -w2w2[3];
    dyad[3][0][1] = -Onkb[3][2];
    dyad[3][0][2] = 0.;
    dyad[3][1][0] = Onkb[3][2];
    dyad[3][1][1] = -w2w2[3];
    dyad[3][1][2] = 0.;
    dyad[3][2][0] = 0.;
    dyad[3][2][1] = 0.;
    dyad[3][2][2] = 0.;
    dyad[4][0][0] = -w2w2[4];
    dyad[4][0][1] = -Onkb[4][2];
    dyad[4][0][2] = 0.;
    dyad[4][1][0] = Onkb[4][2];
    dyad[4][1][1] = -w2w2[4];
    dyad[4][1][2] = 0.;
    dyad[4][2][0] = 0.;
    dyad[4][2][1] = 0.;
    dyad[4][2][2] = 0.;
/*
Compute ank & anb (mass center linear accels in N)
*/
    Ankb[0][0] = (rk[0][1]*udot[0]);
    AOnkri[1][0] = (Ankb[0][0]-(ri[1][1]*udot[0]));
    Ankb[1][0] = ((AOnkri[1][0]*c1)+(Onkb[1][2]*rk[1][1]));
    Ankb[1][1] = -(AOnkri[1][0]*s1);
    AOnkri[2][0] = (Ankb[1][0]-(Onkb[1][2]*ri[2][1]));
    Ankb[2][0] = ((Onkb[2][2]*rk[2][1])+((Ankb[1][1]*s2)+(AOnkri[2][0]*c2)));
    Ankb[2][1] = ((Ankb[1][1]*c2)-(AOnkri[2][0]*s2));
    AOnkri[3][0] = (Ankb[2][0]-(Onkb[2][2]*ri[3][1]));
    Ankb[3][0] = ((Onkb[3][2]*rk[3][1])+((Ankb[2][1]*s3)+(AOnkri[3][0]*c3)));
    Ankb[3][1] = ((Ankb[2][1]*c3)-(AOnkri[3][0]*s3));
    AOnkri[4][0] = (Ankb[3][0]-(Onkb[3][2]*ri[4][1]));
    Ankb[4][0] = ((Onkb[4][2]*rk[4][1])+((Ankb[3][1]*s4)+(AOnkri[4][0]*c4)));
    Ankb[4][1] = ((Ankb[3][1]*c4)-(AOnkri[4][0]*s4));
    ank[0][0] = ((Ankb[0][0]*c0)-(Atk[0][1]*s0));
    ank[0][1] = ((Ankb[0][0]*s0)+(Atk[0][1]*c0));
    AnkAtk[1][0] = (Ankb[1][0]+Atk[1][0]);
    AnkAtk[1][1] = (Ankb[1][1]+Atk[1][1]);
    ank[1][0] = ((AnkAtk[1][0]*cnk[1][0][0])+(AnkAtk[1][1]*cnk[1][0][1]));
    ank[1][1] = ((AnkAtk[1][0]*cnk[1][1][0])+(AnkAtk[1][1]*cnk[1][1][1]));
    AnkAtk[2][0] = (Ankb[2][0]+Atk[2][0]);
    AnkAtk[2][1] = (Ankb[2][1]+Atk[2][1]);
    ank[2][0] = ((AnkAtk[2][0]*cnk[2][0][0])+(AnkAtk[2][1]*cnk[2][0][1]));
    ank[2][1] = ((AnkAtk[2][0]*cnk[2][1][0])+(AnkAtk[2][1]*cnk[2][1][1]));
    AnkAtk[3][0] = (Ankb[3][0]+Atk[3][0]);
    AnkAtk[3][1] = (Ankb[3][1]+Atk[3][1]);
    ank[3][0] = ((AnkAtk[3][0]*cnk[3][0][0])+(AnkAtk[3][1]*cnk[3][0][1]));
    ank[3][1] = ((AnkAtk[3][0]*cnk[3][1][0])+(AnkAtk[3][1]*cnk[3][1][1]));
    AnkAtk[4][0] = (Ankb[4][0]+Atk[4][0]);
    AnkAtk[4][1] = (Ankb[4][1]+Atk[4][1]);
    ank[4][0] = ((AnkAtk[4][0]*cnk[4][0][0])+(AnkAtk[4][1]*cnk[4][0][1]));
    ank[4][1] = ((AnkAtk[4][0]*cnk[4][1][0])+(AnkAtk[4][1]*cnk[4][1][1]));
    anb[0][0] = ank[0][0];
    anb[0][1] = ank[0][1];
    anb[0][2] = 0.;
    anb[1][0] = ank[1][0];
    anb[1][1] = ank[1][1];
    anb[1][2] = 0.;
    anb[2][0] = ank[2][0];
    anb[2][1] = ank[2][1];
    anb[2][2] = 0.;
    anb[3][0] = ank[3][0];
    anb[3][1] = ank[3][1];
    anb[3][2] = 0.;
    anb[4][0] = ank[4][0];
    anb[4][1] = ank[4][1];
    anb[4][2] = 0.;
    roustate = 3;
/*
 Used 0.03 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   52 adds/subtracts/negates
                     43 multiplies
                      0 divides
                    115 assignments
*/
}

b1sspseudo(lqout,luout)
    double lqout[1],luout[1];
{
/*
Return pseudo-coordinates for loop joints.

*/
/*
There are no loop joints in this system.

*/
}

b1sspsqdot(lqdout)
    double lqdout[1];
{
/*
Return pseudo-coordinate derivatives for loop joints.

*/
/*
There are no loop joints in this system.

*/
}

b1sspsudot(ludout)
    double ludout[1];
{
/*
Return pseudo-coordinate accelerations for loop joints.

*/
/*
There are no loop joints in this system.

*/
}

b1ssperr(errs)
    double errs[1];
{

}

b1ssverr(errs)
    double errs[1];
{

}

b1ssaerr(errs)
    double errs[1];
{

}
int 
b1ssindx(joint,axis)
    int joint,axis;
{
    int offs,gotit;

    if (b1sschkjaxis(36,joint,axis) != 0) {
        return 0;
    }
    gotit = 0;
    if (jtype[joint] == 4) {
        if (axis == 3) {
            offs = ballq[joint];
            gotit = 1;
        }
    } else {
        if ((jtype[joint] == 6) || (jtype[joint] == 21)) {
            if (axis == 6) {
                offs = ballq[joint];
                gotit = 1;
            }
        }
    }
    if (gotit == 0) {
        offs = firstq[joint]+axis;
    }
    return offs;
}

b1sspresacc(joint,axis,prval)
    int joint,axis;
    double prval;
{

}

b1sspresvel(joint,axis,prval)
    int joint,axis;
    double prval;
{

}

b1ssprespos(joint,axis,prval)
    int joint,axis;
    double prval;
{

}

b1ssgetht(joint,axis,torque)
    int joint,axis;
    double *torque;
{

    if (b1sschkjaxis(30,joint,axis) != 0) {
        return;
    }
    if (roustate != 3) {
        b1ssseterr(30,24);
        return;
    }
    *torque = tauc[b1ssindx(joint,axis)];
}

b1sshinget(joint,axis,torque)
    int joint,axis;
    double torque;
{

    if (b1sschkjaxis(10,joint,axis) != 0) {
        return;
    }
    if (roustate != 2) {
        b1ssseterr(10,23);
        return;
    }
    if (mfrcflg != 0) {
        mtau[b1ssindx(joint,axis)] = mtau[b1ssindx(joint,axis)]+torque;
    } else {
        fs0flg = 0;
        utau[b1ssindx(joint,axis)] = utau[b1ssindx(joint,axis)]+torque;
    }
}

b1sspointf(body,point,force)
    int body;
    double point[3],force[3];
{
    double torque[3];

    if (b1sschkbnum(11,body) != 0) {
        return;
    }
    if (roustate != 2) {
        b1ssseterr(11,23);
        return;
    }
    if (body == -1) {
        return;
    }
    torque[0] = point[1]*force[2]-point[2]*force[1];
    torque[1] = point[2]*force[0]-point[0]*force[2];
    torque[2] = point[0]*force[1]-point[1]*force[0];
    if (mfrcflg != 0) {
        mfk[body][0] = mfk[body][0]+force[0];
        mtk[body][0] = mtk[body][0]+torque[0];
        mfk[body][1] = mfk[body][1]+force[1];
        mtk[body][1] = mtk[body][1]+torque[1];
        mfk[body][2] = mfk[body][2]+force[2];
        mtk[body][2] = mtk[body][2]+torque[2];
    } else {
        fs0flg = 0;
        ufk[body][0] = ufk[body][0]+force[0];
        utk[body][0] = utk[body][0]+torque[0];
        ufk[body][1] = ufk[body][1]+force[1];
        utk[body][1] = utk[body][1]+torque[1];
        ufk[body][2] = ufk[body][2]+force[2];
        utk[body][2] = utk[body][2]+torque[2];
    }
}

b1ssbodyt(body,torque)
    int body;
    double torque[3];
{

    if (b1sschkbnum(12,body) != 0) {
        return;
    }
    if (roustate != 2) {
        b1ssseterr(12,23);
        return;
    }
    if (body == -1) {
        return;
    }
    if (mfrcflg != 0) {
        mtk[body][0] = mtk[body][0]+torque[0];
        mtk[body][1] = mtk[body][1]+torque[1];
        mtk[body][2] = mtk[body][2]+torque[2];
    } else {
        fs0flg = 0;
        utk[body][0] = utk[body][0]+torque[0];
        utk[body][1] = utk[body][1]+torque[1];
        utk[body][2] = utk[body][2]+torque[2];
    }
}

b1ssdoww(routine)
    int routine;
{

    roustate = 2;
    if (wwflg == 0) {
        wwflg = 1;
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      0 assignments
*/
}

b1ssxudot0(routine,oudot0)
    int routine;
    double oudot0[5];
{
/*
Compute unconstrained equations
*/
    int i;

    b1sslhs(routine);
/*
Solve equations for udots
*/
    b1ssfs0();
    b1ssldubslv(5,5,mmap,works,mlo,mdi,fs,udot);
    for (i = 0; i <= 4; i++) {
        oudot0[i] = udot[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      5 assignments
*/
}

b1ssudot0(oudot0)
    double oudot0[5];
{

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(66,23);
        return;
    }
    b1ssxudot0(66,oudot0);
}

b1sssetudot(iudot)
    double iudot[5];
{
/*
Assign udots and advance to stage Dynamics Ready
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(68,23);
        return;
    }
    for (i = 0; i <= 4; i++) {
        udot[i] = iudot[i];
    }
    b1ssrhs();
}

b1ssxudotm(routine,imult,oudotm)
    int routine;
    double imult[1],oudotm[5];
{
/*
Compute udots due only to multipliers
*/
    int i;

    b1sslhs(routine);
    for (i = 0; i <= 4; i++) {
        udot[i] = 0.;
    }
    for (i = 0; i <= 4; i++) {
        oudotm[i] = udot[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     10 assignments
*/
}

b1ssudotm(imult,oudotm)
    double imult[1],oudotm[5];
{

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(67,23);
        return;
    }
    b1ssxudotm(67,imult,oudotm);
}

b1ssderiv(oqdot,oudot)
    double oqdot[5],oudot[5];
{
/*
This is the derivative section for a 5-body ground-based
system with 5 hinge degree(s) of freedom.
*/
    int i;
    double udot0[5],udot1[5];

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(17,23);
        return;
    }
    if (stabvelq == 1) {
        b1ssseterr(17,32);
    }
    if (stabposq == 1) {
        b1ssseterr(17,33);
    }
    wsiz = 0;
/*
Compute unconstrained equations
*/
    b1ssxudot0(17,udot0);
    b1ssrhs();
    wrank = 0;
    for (i = 0; i <= 4; i++) {
        oqdot[i] = qdot[i];
    }
    for (i = 0; i <= 4; i++) {
        oudot[i] = udot[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     10 assignments
*/
}
/*
Compute residuals for use with DAE integrator
*/

b1ssresid(eqdot,eudot,emults,resid)
    double eqdot[5],eudot[5],emults[1],resid[10];
{
    int i;
    double uderrs[5];

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(16,23);
        return;
    }
    if (stabposq == 1) {
        b1ssseterr(16,33);
    }
    b1ssfulltrq(eudot,emults,uderrs);
    for (i = 0; i < 5; i++) {
        resid[i] = eqdot[i]-qdot[i];
    }
    for (i = 0; i < 5; i++) {
        resid[5+i] = uderrs[i];
    }
    for (i = 0; i < 5; i++) {
        udot[i] = eudot[i];
    }
    b1ssrhs();
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    5 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     15 assignments
*/
}

b1ssmult(omults,owrank,omultmap)
    double omults[1];
    int *owrank,omultmap[1];
{

    if (roustate != 3) {
        b1ssseterr(34,24);
        return;
    }
    *owrank = wrank;
}

b1ssreac(force,torque)
    double force[5][3],torque[5][3];
{
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

    if (roustate != 3) {
        b1ssseterr(31,24);
        return;
    }
/*
Compute reaction forces for non-weld tree joints
*/
    fc[4][0] = ((mk[4]*(AnkAtk[4][0]-gk[4][0]))-ufk[4][0]);
    fc[4][1] = ((mk[4]*(AnkAtk[4][1]-gk[4][1]))-ufk[4][1]);
    fc[4][2] = -ufk[4][2];
    tc[4][0] = -(utk[4][0]+(fc[4][2]*rk[4][1]));
    tc[4][1] = -utk[4][1];
    tc[4][2] = ((ik[4][2][2]*Onkb[4][2])-(utk[4][2]-(fc[4][0]*rk[4][1])));
    fccikt[4][0] = ((fc[4][0]*c4)-(fc[4][1]*s4));
    fccikt[4][1] = ((fc[4][0]*s4)+(fc[4][1]*c4));
    fccikt[4][2] = fc[4][2];
    ffk[3][0] = (ufk[3][0]-fccikt[4][0]);
    ffk[3][1] = (ufk[3][1]-fccikt[4][1]);
    ffk[3][2] = (ufk[3][2]-fccikt[4][2]);
    ttk[3][0] = (utk[3][0]-((fccikt[4][2]*ri[4][1])+((tc[4][0]*c4)-(tc[4][1]*s4)
      )));
    ttk[3][1] = (utk[3][1]-((tc[4][0]*s4)+(tc[4][1]*c4)));
    ttk[3][2] = (utk[3][2]-(tc[4][2]-(fccikt[4][0]*ri[4][1])));
    fc[3][0] = ((mk[3]*(AnkAtk[3][0]-gk[3][0]))-ffk[3][0]);
    fc[3][1] = ((mk[3]*(AnkAtk[3][1]-gk[3][1]))-ffk[3][1]);
    fc[3][2] = -ffk[3][2];
    tc[3][0] = -(ttk[3][0]+(fc[3][2]*rk[3][1]));
    tc[3][1] = -ttk[3][1];
    tc[3][2] = ((ik[3][2][2]*Onkb[3][2])-(ttk[3][2]-(fc[3][0]*rk[3][1])));
    fccikt[3][0] = ((fc[3][0]*c3)-(fc[3][1]*s3));
    fccikt[3][1] = ((fc[3][0]*s3)+(fc[3][1]*c3));
    fccikt[3][2] = fc[3][2];
    ffk[2][0] = (ufk[2][0]-fccikt[3][0]);
    ffk[2][1] = (ufk[2][1]-fccikt[3][1]);
    ffk[2][2] = (ufk[2][2]-fccikt[3][2]);
    ttk[2][0] = (utk[2][0]-((fccikt[3][2]*ri[3][1])+((tc[3][0]*c3)-(tc[3][1]*s3)
      )));
    ttk[2][1] = (utk[2][1]-((tc[3][0]*s3)+(tc[3][1]*c3)));
    ttk[2][2] = (utk[2][2]-(tc[3][2]-(fccikt[3][0]*ri[3][1])));
    fc[2][0] = ((mk[2]*(AnkAtk[2][0]-gk[2][0]))-ffk[2][0]);
    fc[2][1] = ((mk[2]*(AnkAtk[2][1]-gk[2][1]))-ffk[2][1]);
    fc[2][2] = -ffk[2][2];
    tc[2][0] = -(ttk[2][0]+(fc[2][2]*rk[2][1]));
    tc[2][1] = -ttk[2][1];
    tc[2][2] = ((ik[2][2][2]*Onkb[2][2])-(ttk[2][2]-(fc[2][0]*rk[2][1])));
    fccikt[2][0] = ((fc[2][0]*c2)-(fc[2][1]*s2));
    fccikt[2][1] = ((fc[2][0]*s2)+(fc[2][1]*c2));
    fccikt[2][2] = fc[2][2];
    ffk[1][0] = (ufk[1][0]-fccikt[2][0]);
    ffk[1][1] = (ufk[1][1]-fccikt[2][1]);
    ffk[1][2] = (ufk[1][2]-fccikt[2][2]);
    ttk[1][0] = (utk[1][0]-((fccikt[2][2]*ri[2][1])+((tc[2][0]*c2)-(tc[2][1]*s2)
      )));
    ttk[1][1] = (utk[1][1]-((tc[2][0]*s2)+(tc[2][1]*c2)));
    ttk[1][2] = (utk[1][2]-(tc[2][2]-(fccikt[2][0]*ri[2][1])));
    fc[1][0] = ((mk[1]*(AnkAtk[1][0]-gk[1][0]))-ffk[1][0]);
    fc[1][1] = ((mk[1]*(AnkAtk[1][1]-gk[1][1]))-ffk[1][1]);
    fc[1][2] = -ffk[1][2];
    tc[1][0] = -(ttk[1][0]+(fc[1][2]*rk[1][1]));
    tc[1][1] = -ttk[1][1];
    tc[1][2] = ((ik[1][2][2]*Onkb[1][2])-(ttk[1][2]-(fc[1][0]*rk[1][1])));
    fccikt[1][0] = ((fc[1][0]*c1)-(fc[1][1]*s1));
    fccikt[1][1] = ((fc[1][0]*s1)+(fc[1][1]*c1));
    fccikt[1][2] = fc[1][2];
    ffk[0][0] = (ufk[0][0]-fccikt[1][0]);
    ffk[0][1] = (ufk[0][1]-fccikt[1][1]);
    ffk[0][2] = (ufk[0][2]-fccikt[1][2]);
    ttk[0][0] = (utk[0][0]-((fccikt[1][2]*ri[1][1])+((tc[1][0]*c1)-(tc[1][1]*s1)
      )));
    ttk[0][1] = (utk[0][1]-((tc[1][0]*s1)+(tc[1][1]*c1)));
    ttk[0][2] = (utk[0][2]-(tc[1][2]-(fccikt[1][0]*ri[1][1])));
    fc[0][0] = ((mk[0]*(Ankb[0][0]+(9.81*s0)))-ffk[0][0]);
    fc[0][1] = ((mk[0]*(Atk[0][1]+(9.81*c0)))-ffk[0][1]);
    fc[0][2] = -ffk[0][2];
    tc[0][0] = -(ttk[0][0]+(fc[0][2]*rk[0][1]));
    tc[0][1] = -ttk[0][1];
    tc[0][2] = ((ik[0][2][2]*udot[0])-(ttk[0][2]-(fc[0][0]*rk[0][1])));
    force[0][0] = fc[0][0];
    torque[0][0] = tc[0][0];
    force[0][1] = fc[0][1];
    torque[0][1] = tc[0][1];
    force[0][2] = fc[0][2];
    torque[0][2] = tc[0][2];
    force[1][0] = fc[1][0];
    torque[1][0] = tc[1][0];
    force[1][1] = fc[1][1];
    torque[1][1] = tc[1][1];
    force[1][2] = fc[1][2];
    torque[1][2] = tc[1][2];
    force[2][0] = fc[2][0];
    torque[2][0] = tc[2][0];
    force[2][1] = fc[2][1];
    torque[2][1] = tc[2][1];
    force[2][2] = fc[2][2];
    torque[2][2] = tc[2][2];
    force[3][0] = fc[3][0];
    torque[3][0] = tc[3][0];
    force[3][1] = fc[3][1];
    torque[3][1] = tc[3][1];
    force[3][2] = fc[3][2];
    torque[3][2] = tc[3][2];
    force[4][0] = fc[4][0];
    torque[4][0] = tc[4][0];
    force[4][1] = fc[4][1];
    torque[4][1] = tc[4][1];
    force[4][2] = fc[4][2];
    torque[4][2] = tc[4][2];
/*
Compute reaction forces for tree weld joints
*/
/*
 Used 0.04 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain   98 adds/subtracts/negates
                     67 multiplies
                      0 divides
                     96 assignments
*/
}

b1ssmom(lm,am,ke)
    double lm[3],am[3],*ke;
{
/*
Compute system linear and angular momentum, and kinetic energy.

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
    double lk[5][3],hnk[5][3];

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(19,23);
        return;
    }
    lk[0][0] = (mk[0]*vnk[0][0]);
    lk[0][1] = (mk[0]*vnk[0][1]);
    lk[1][0] = (mk[1]*vnk[1][0]);
    lk[1][1] = (mk[1]*vnk[1][1]);
    lk[2][0] = (mk[2]*vnk[2][0]);
    lk[2][1] = (mk[2]*vnk[2][1]);
    lk[3][0] = (mk[3]*vnk[3][0]);
    lk[3][1] = (mk[3]*vnk[3][1]);
    lk[4][0] = (mk[4]*vnk[4][0]);
    lk[4][1] = (mk[4]*vnk[4][1]);
    hnk[0][2] = (ik[0][2][2]*u[0]);
    hnk[1][2] = (ik[1][2][2]*wk[1][2]);
    hnk[2][2] = (ik[2][2][2]*wk[2][2]);
    hnk[3][2] = (ik[3][2][2]*wk[3][2]);
    hnk[4][2] = (ik[4][2][2]*wk[4][2]);
    lm[0] = (lk[4][0]+(lk[3][0]+(lk[2][0]+(lk[0][0]+lk[1][0]))));
    lm[1] = (lk[4][1]+(lk[3][1]+(lk[2][1]+(lk[0][1]+lk[1][1]))));
    lm[2] = 0.;
    am[0] = 0.;
    am[1] = 0.;
    am[2] = (((hnk[4][2]+((lk[4][1]*rnk[4][0])-(lk[4][0]*rnk[4][1])))+((
      hnk[3][2]+((lk[3][1]*rnk[3][0])-(lk[3][0]*rnk[3][1])))+((hnk[2][2]+((
      lk[2][1]*rnk[2][0])-(lk[2][0]*rnk[2][1])))+((hnk[0][2]+((lk[0][1]*
      rnk[0][0])-(lk[0][0]*rnk[0][1])))+(hnk[1][2]+((lk[1][1]*rnk[1][0])-(
      lk[1][0]*rnk[1][1])))))))-((com[0]*lm[1])-(com[1]*lm[0])));
    *ke = (.5*(((hnk[4][2]*wk[4][2])+((lk[4][0]*vnk[4][0])+(lk[4][1]*vnk[4][1]))
      )+(((hnk[3][2]*wk[3][2])+((lk[3][0]*vnk[3][0])+(lk[3][1]*vnk[3][1])))+(((
      hnk[2][2]*wk[2][2])+((lk[2][0]*vnk[2][0])+(lk[2][1]*vnk[2][1])))+(((
      hnk[0][2]*u[0])+((lk[0][0]*vnk[0][0])+(lk[0][1]*vnk[0][1])))+((hnk[1][2]*
      wk[1][2])+((lk[1][0]*vnk[1][0])+(lk[1][1]*vnk[1][1]))))))));
/*
 Used 0.02 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain   38 adds/subtracts/negates
                     43 multiplies
                      0 divides
                     22 assignments
*/
}

b1sssys(mtoto,cm,icm)
    double *mtoto,cm[3],icm[3][3];
{
/*
Compute system total mass, and instantaneous center of mass and
inertia matrix.

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
    double ikcnkt[5][3][3];

    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(20,23);
        return;
    }
    *mtoto = mtot;
    cm[0] = com[0];
    cm[1] = com[1];
    cm[2] = 0.;
    temp[0] = (((mk[3]*(rnk[3][1]*rnk[3][1]))+((cnk[3][0][0]*cnk[3][0][0])+(
      cnk[3][0][1]*cnk[3][0][1])))+(((mk[0]*(rnk[0][1]*rnk[0][1]))+((mk[1]*(
      rnk[1][1]*rnk[1][1]))+((cnk[1][0][0]*cnk[1][0][0])+(cnk[1][0][1]*
      cnk[1][0][1]))))+((mk[2]*(rnk[2][1]*rnk[2][1]))+((cnk[2][0][0]*
      cnk[2][0][0])+(cnk[2][0][1]*cnk[2][0][1])))));
    icm[0][0] = (1.+((((mk[4]*(rnk[4][1]*rnk[4][1]))+((cnk[4][0][0]*cnk[4][0][0]
      )+(cnk[4][0][1]*cnk[4][0][1])))+temp[0])-(mtot*(com[1]*com[1]))));
    temp[0] = ((((cnk[3][0][0]*cnk[3][1][0])+(cnk[3][0][1]*cnk[3][1][1]))-(mk[3]
      *(rnk[3][0]*rnk[3][1])))+((((cnk[2][0][0]*cnk[2][1][0])+(cnk[2][0][1]*
      cnk[2][1][1]))-(mk[2]*(rnk[2][0]*rnk[2][1])))+((((cnk[1][0][0]*
      cnk[1][1][0])+(cnk[1][0][1]*cnk[1][1][1]))-(mk[1]*(rnk[1][0]*rnk[1][1])))-
      (mk[0]*(rnk[0][0]*rnk[0][1])))));
    icm[0][1] = ((mtot*(com[0]*com[1]))+((((cnk[4][0][0]*cnk[4][1][0])+(
      cnk[4][0][1]*cnk[4][1][1]))-(mk[4]*(rnk[4][0]*rnk[4][1])))+temp[0]));
    icm[0][2] = 0.;
    icm[1][0] = icm[0][1];
    temp[0] = (((mk[3]*(rnk[3][0]*rnk[3][0]))+((cnk[3][1][0]*cnk[3][1][0])+(
      cnk[3][1][1]*cnk[3][1][1])))+(((mk[0]*(rnk[0][0]*rnk[0][0]))+((mk[1]*(
      rnk[1][0]*rnk[1][0]))+((cnk[1][1][0]*cnk[1][1][0])+(cnk[1][1][1]*
      cnk[1][1][1]))))+((mk[2]*(rnk[2][0]*rnk[2][0]))+((cnk[2][1][0]*
      cnk[2][1][0])+(cnk[2][1][1]*cnk[2][1][1])))));
    icm[1][1] = (1.+((((mk[4]*(rnk[4][0]*rnk[4][0]))+((cnk[4][1][0]*cnk[4][1][0]
      )+(cnk[4][1][1]*cnk[4][1][1])))+temp[0])-(mtot*(com[0]*com[0]))));
    icm[1][2] = 0.;
    icm[2][0] = icm[0][2];
    icm[2][1] = icm[1][2];
    icm[2][2] = (((ik[4][2][2]+(mk[4]*((rnk[4][0]*rnk[4][0])+(rnk[4][1]*
      rnk[4][1]))))+((ik[3][2][2]+(mk[3]*((rnk[3][0]*rnk[3][0])+(rnk[3][1]*
      rnk[3][1]))))+((ik[2][2][2]+(mk[2]*((rnk[2][0]*rnk[2][0])+(rnk[2][1]*
      rnk[2][1]))))+((ik[0][2][2]+(mk[0]*((rnk[0][0]*rnk[0][0])+(rnk[0][1]*
      rnk[0][1]))))+(ik[1][2][2]+(mk[1]*((rnk[1][0]*rnk[1][0])+(rnk[1][1]*
      rnk[1][1]))))))))-(mtot*((com[0]*com[0])+(com[1]*com[1]))));
/*
 Used 0.05 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain   57 adds/subtracts/negates
                     78 multiplies
                      0 divides
                     16 assignments
*/
}

b1sspos(body,pt,loc)
    int body;
    double pt[3],loc[3];
{
/*
Return inertial frame location of a point on a body.

*/
    double pv[3];

    if (b1sschkbnum(21,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(21,23);
        return;
    }
    if (body  ==  -1) {
        loc[0] = pt[0];
        loc[1] = pt[1];
        loc[2] = pt[2];
    } else {
        pv[0] = rnb[body][0]+pt[0]*cnb[body][0][0]+pt[1]*cnb[body][0][1]+pt[2]*
          cnb[body][0][2];
        pv[1] = rnb[body][1]+pt[0]*cnb[body][1][0]+pt[1]*cnb[body][1][1]+pt[2]*
          cnb[body][1][2];
        pv[2] = rnb[body][2]+pt[0]*cnb[body][2][0]+pt[1]*cnb[body][2][1]+pt[2]*
          cnb[body][2][2];
        loc[0] = pv[0];
        loc[1] = pv[1];
        loc[2] = pv[2];
    }
}

b1ssvel(body,pt,velo)
    int body;
    double pt[3],velo[3];
{
/*
Return inertial frame velocity of a point on a body.

*/
    double pv[3];

    if (b1sschkbnum(22,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(22,23);
        return;
    }
    if (body  ==  -1) {
        velo[0] = 0.;
        velo[1] = 0.;
        velo[2] = 0.;
    } else {
        pv[0] = wb[body][1]*pt[2]-wb[body][2]*pt[1];
        pv[1] = wb[body][2]*pt[0]-wb[body][0]*pt[2];
        pv[2] = wb[body][0]*pt[1]-wb[body][1]*pt[0];
        velo[0] = vnb[body][0]+pv[0]*cnb[body][0][0]+pv[1]*cnb[body][0][1]+pv[2]
          *cnb[body][0][2];
        velo[1] = vnb[body][1]+pv[0]*cnb[body][1][0]+pv[1]*cnb[body][1][1]+pv[2]
          *cnb[body][1][2];
        velo[2] = vnb[body][2]+pv[0]*cnb[body][2][0]+pv[1]*cnb[body][2][1]+pv[2]
          *cnb[body][2][2];
    }
}

b1ssorient(body,dircos)
    int body;
    double dircos[3][3];
{
/*
Return orientation of body w.r.t. ground frame.

*/

    if (b1sschkbnum(23,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(23,23);
        return;
    }
    if (body == -1) {
        dircos[0][0] = 1.;
        dircos[0][1] = 0.;
        dircos[0][2] = 0.;
        dircos[1][0] = 0.;
        dircos[1][1] = 1.;
        dircos[1][2] = 0.;
        dircos[2][0] = 0.;
        dircos[2][1] = 0.;
        dircos[2][2] = 1.;
    } else {
        dircos[0][0] = cnb[body][0][0];
        dircos[0][1] = cnb[body][0][1];
        dircos[0][2] = cnb[body][0][2];
        dircos[1][0] = cnb[body][1][0];
        dircos[1][1] = cnb[body][1][1];
        dircos[1][2] = cnb[body][1][2];
        dircos[2][0] = cnb[body][2][0];
        dircos[2][1] = cnb[body][2][1];
        dircos[2][2] = cnb[body][2][2];
    }
}

b1ssangvel(body,avel)
    int body;
    double avel[3];
{
/*
Return angular velocity of the body.

*/

    if (b1sschkbnum(24,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(24,23);
        return;
    }
    if (body == -1) {
        avel[0] = 0.;
        avel[1] = 0.;
        avel[2] = 0.;
    } else {
        avel[0] = wb[body][0];
        avel[1] = wb[body][1];
        avel[2] = wb[body][2];
    }
}

b1sstrans(frbod,ivec,tobod,ovec)
    int frbod;
    double ivec[3];
    int tobod;
    double ovec[3];
{
/*
Transform ivec from frbod frame to tobod frame.

*/
    double pv[3];

    if (b1sschkbnum(25,frbod) != 0) {
        return;
    }
    if (b1sschkbnum(25,tobod) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(25,23);
        return;
    }
    if (frbod == tobod) {
        b1ssvcopy(ivec,ovec);
        return;
    }
    if (frbod == -1) {
        b1ssvcopy(ivec,pv);
        ovec[0] = pv[0]*cnb[tobod][0][0]+pv[1]*cnb[tobod][1][0]+pv[2]*cnb[tobod
          ][2][0];
        ovec[1] = pv[0]*cnb[tobod][0][1]+pv[1]*cnb[tobod][1][1]+pv[2]*cnb[tobod
          ][2][1];
        ovec[2] = pv[0]*cnb[tobod][0][2]+pv[1]*cnb[tobod][1][2]+pv[2]*cnb[tobod
          ][2][2];
        return;
    }
    if (tobod == -1) {
        b1ssvcopy(ivec,pv);
        ovec[0] = pv[0]*cnb[frbod][0][0]+pv[1]*cnb[frbod][0][1]+pv[2]*cnb[frbod
          ][0][2];
        ovec[1] = pv[0]*cnb[frbod][1][0]+pv[1]*cnb[frbod][1][1]+pv[2]*cnb[frbod
          ][1][2];
        ovec[2] = pv[0]*cnb[frbod][2][0]+pv[1]*cnb[frbod][2][1]+pv[2]*cnb[frbod
          ][2][2];
        return;
    }
    pv[0] = ivec[0]*cnb[frbod][0][0]+ivec[1]*cnb[frbod][0][1]+ivec[2]*cnb[frbod
      ][0][2];
    pv[1] = ivec[0]*cnb[frbod][1][0]+ivec[1]*cnb[frbod][1][1]+ivec[2]*cnb[frbod
      ][1][2];
    pv[2] = ivec[0]*cnb[frbod][2][0]+ivec[1]*cnb[frbod][2][1]+ivec[2]*cnb[frbod
      ][2][2];
    ovec[0] = pv[0]*cnb[tobod][0][0]+pv[1]*cnb[tobod][1][0]+pv[2]*cnb[tobod][2][
      0];
    ovec[1] = pv[0]*cnb[tobod][0][1]+pv[1]*cnb[tobod][1][1]+pv[2]*cnb[tobod][2][
      1];
    ovec[2] = pv[0]*cnb[tobod][0][2]+pv[1]*cnb[tobod][1][2]+pv[2]*cnb[tobod][2][
      2];
}

b1ssrel2cart(coord,body,point,linchg,rotchg)
    int coord,body;
    double point[3],linchg[3],rotchg[3];
{
/* Return derivative of pt loc and body orient w.r.t. hinge rate
*/
    int x,i,gnd;
    double lin[3],pv[3];

    if ((coord < 0) || (coord > 4)) {
        b1ssseterr(59,45);
        return;
    }
    if (b1sschkbnum(59,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1ssseterr(59,23);
        return;
    }
    gnd = -1;
    if (body == gnd) {
        x = -1;
    } else {
        x = firstq[body]+njntdof[body]-1;
    }
    if (x < coord) {
        b1ssvset(0.,0.,0.,linchg);
        b1ssvset(0.,0.,0.,rotchg);
        return;
    }
    b1ssdovpk();
    for (i = 0; i < 3; i++) {
        rotchg[i] = Wpk[coord][x][i];
        lin[i] = Vpk[coord][x][i];
    }
    if (body == gnd) {
        b1ssvcopy(point,pv);
    } else {
        pv[0] = rcom[body][0]+point[0];
        pv[1] = rcom[body][1]+point[1];
        pv[2] = rcom[body][2]+point[2];
    }
    b1ssvcross(rotchg,pv,linchg);
    b1ssvadd(linchg,lin,linchg);
}

b1ssacc(body,pt,accel)
    int body;
    double pt[3],accel[3];
{
/*
Return linear acceleration a point of the specified body.

*/
    double pv[3];

    if (b1sschkbnum(32,body) != 0) {
        return;
    }
    if (roustate != 3) {
        b1ssseterr(32,24);
        return;
    }
    if (body  ==  -1) {
        accel[0] = 0.;
        accel[1] = 0.;
        accel[2] = 0.;
    } else {
        pv[0] = pt[0]*dyad[body][0][0]+pt[1]*dyad[body][0][1]+pt[2]*dyad[body][0
          ][2];
        pv[1] = pt[0]*dyad[body][1][0]+pt[1]*dyad[body][1][1]+pt[2]*dyad[body][1
          ][2];
        pv[2] = pt[0]*dyad[body][2][0]+pt[1]*dyad[body][2][1]+pt[2]*dyad[body][2
          ][2];
        accel[0] = anb[body][0]+pv[0]*cnb[body][0][0]+pv[1]*cnb[body][0][1]+pv[2
          ]*cnb[body][0][2];
        accel[1] = anb[body][1]+pv[0]*cnb[body][1][0]+pv[1]*cnb[body][1][1]+pv[2
          ]*cnb[body][1][2];
        accel[2] = anb[body][2]+pv[0]*cnb[body][2][0]+pv[1]*cnb[body][2][1]+pv[2
          ]*cnb[body][2][2];
    }
}

b1ssangacc(body,aacc)
    int body;
    double aacc[3];
{
/*
Return angular acceleration of the body.

*/

    if (b1sschkbnum(33,body) != 0) {
        return;
    }
    if (roustate != 3) {
        b1ssseterr(33,24);
        return;
    }
    if (body == -1) {
        aacc[0] = 0.;
        aacc[1] = 0.;
        aacc[2] = 0.;
    } else {
        aacc[0] = onb[body][0];
        aacc[1] = onb[body][1];
        aacc[2] = onb[body][2];
    }
}

b1ssgrav(gravin)
    double gravin[3];
{

    b1ssseterr(1,19);
    roustate = 0;
}

b1ssmass(body,massin)
    int body;
    double massin;
{

    if (b1sschkbnum(2,body) != 0) {
        return;
    }
    if (body == -1) {
        b1ssseterr(2,15);
        return;
    }
    if (mkq[body] != 0) {
        mk[body] = massin;
        mkq[body] = 3;
    } else {
        b1ssseterr(2,19);
    }
    roustate = 0;
}

b1ssiner(body,inerin)
    int body;
    double inerin[3][3];
{
    int anyques;

    if (b1sschkbnum(3,body) != 0) {
        return;
    }
    if (body == -1) {
        b1ssseterr(3,15);
        return;
    }
    anyques = 0;
    if (ikq[body][0][0]  !=  0) {
        ik[body][0][0] = inerin[0][0];
        ikq[body][0][0] = 3;
        anyques = 1;
    }
    if (ikq[body][0][1]  !=  0) {
        ik[body][0][1] = inerin[0][1];
        ikq[body][0][1] = 3;
        ik[body][1][0] = inerin[0][1];
        ikq[body][1][0] = 3;
        anyques = 1;
    }
    if (ikq[body][0][2]  !=  0) {
        ik[body][0][2] = inerin[0][2];
        ikq[body][0][2] = 3;
        ik[body][2][0] = inerin[0][2];
        ikq[body][2][0] = 3;
        anyques = 1;
    }
    if (ikq[body][1][1]  !=  0) {
        ik[body][1][1] = inerin[1][1];
        ikq[body][1][1] = 3;
        anyques = 1;
    }
    if (ikq[body][1][2]  !=  0) {
        ik[body][1][2] = inerin[1][2];
        ikq[body][1][2] = 3;
        ik[body][2][1] = inerin[1][2];
        ikq[body][2][1] = 3;
        anyques = 1;
    }
    if (ikq[body][2][2]  !=  0) {
        ik[body][2][2] = inerin[2][2];
        ikq[body][2][2] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        b1ssseterr(3,19);
    }
    roustate = 0;
}

b1ssbtj(joint,btjin)
    int joint;
    double btjin[3];
{
    int anyques;

    if (b1sschkjnum(4,joint) != 0) {
        return;
    }
    anyques = 0;
    if (rkq[joint][0]  !=  0) {
        rk[joint][0] = btjin[0];
        rkq[joint][0] = 3;
        anyques = 1;
    }
    if (rkq[joint][1]  !=  0) {
        rk[joint][1] = btjin[1];
        rkq[joint][1] = 3;
        anyques = 1;
    }
    if (rkq[joint][2]  !=  0) {
        rk[joint][2] = btjin[2];
        rkq[joint][2] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        b1ssseterr(4,19);
    }
    roustate = 0;
}

b1ssitj(joint,itjin)
    int joint;
    double itjin[3];
{
    int anyques;

    if (b1sschkjnum(5,joint) != 0) {
        return;
    }
    anyques = 0;
    if (riq[joint][0]  !=  0) {
        ri[joint][0] = itjin[0];
        riq[joint][0] = 3;
        anyques = 1;
    }
    if (riq[joint][1]  !=  0) {
        ri[joint][1] = itjin[1];
        riq[joint][1] = 3;
        anyques = 1;
    }
    if (riq[joint][2]  !=  0) {
        ri[joint][2] = itjin[2];
        riq[joint][2] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        b1ssseterr(5,19);
    }
    roustate = 0;
}

b1sspin(joint,pinno,pinin)
    int joint,pinno;
    double pinin[3];
{
    int anyques,offs;

    if (b1sschkjpin(6,joint,pinno) != 0) {
        return;
    }
    anyques = 0;
    offs = firstq[joint]+pinno;
    if (jtype[joint] == 21) {
        offs = offs+3;
    }
    if (jtype[joint] == 11) {
        offs = offs+1;
    }
    if (pinq[offs][0]  !=  0) {
        pin[offs][0] = pinin[0];
        pinq[offs][0] = 3;
        anyques = 1;
    }
    if (pinq[offs][1]  !=  0) {
        pin[offs][1] = pinin[1];
        pinq[offs][1] = 3;
        anyques = 1;
    }
    if (pinq[offs][2]  !=  0) {
        pin[offs][2] = pinin[2];
        pinq[offs][2] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        b1ssseterr(6,19);
    }
    roustate = 0;
}

b1sspres(joint,axis,presin)
    int joint,axis,presin;
{
    int anyques;

    if (b1sschkjaxis(37,joint,axis) != 0) {
        return;
    }
    if ((presin != 0) && (presin != 1)) {
        b1ssseterr(37,20);
    }
    anyques = 0;
    if (presq[b1ssindx(joint,axis)]  !=  0) {
        if (presin  !=  0) {
            pres[b1ssindx(joint,axis)] = 1.;
        } else {
            pres[b1ssindx(joint,axis)] = 0.;
        }
        presq[b1ssindx(joint,axis)] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        b1ssseterr(37,19);
    }
    wwflg = 0;
}

b1ssstab(velin,posin)
    double velin,posin;
{

    stabvel = velin;
    stabvelq = 3;
    stabpos = posin;
    stabposq = 3;
}

b1ssgetgrav(gravout)
    double gravout[3];
{

    gravout[0] = grav[0];
    gravout[1] = grav[1];
    gravout[2] = grav[2];
}

b1ssgetmass(body,massout)
    int body;
    double *massout;
{

    if (b1sschkbnum(40,body) != 0) {
        return;
    }
    if (body == -1) {
        b1ssseterr(40,15);
        return;
    }
    *massout = mk[body];
}

b1ssgetiner(body,inerout)
    int body;
    double inerout[3][3];
{

    if (b1sschkbnum(41,body) != 0) {
        return;
    }
    if (body == -1) {
        b1ssseterr(41,15);
        return;
    }
    inerout[0][0] = ik[body][0][0];
    inerout[0][1] = ik[body][0][1];
    inerout[0][2] = ik[body][0][2];
    inerout[1][0] = ik[body][1][0];
    inerout[1][1] = ik[body][1][1];
    inerout[1][2] = ik[body][1][2];
    inerout[2][0] = ik[body][2][0];
    inerout[2][1] = ik[body][2][1];
    inerout[2][2] = ik[body][2][2];
}

b1ssgetbtj(joint,btjout)
    int joint;
    double btjout[3];
{

    if (b1sschkjnum(42,joint) != 0) {
        return;
    }
    btjout[0] = rk[joint][0];
    btjout[1] = rk[joint][1];
    btjout[2] = rk[joint][2];
}

b1ssgetitj(joint,itjout)
    int joint;
    double itjout[3];
{

    if (b1sschkjnum(43,joint) != 0) {
        return;
    }
    itjout[0] = ri[joint][0];
    itjout[1] = ri[joint][1];
    itjout[2] = ri[joint][2];
}

b1ssgetpin(joint,pinno,pinout)
    int joint,pinno;
    double pinout[3];
{
    int offs;

    if (b1sschkjpin(44,joint,pinno) != 0) {
        return;
    }
    offs = firstq[joint]+pinno;
    if (jtype[joint] == 21) {
        offs = offs+3;
    }
    if (jtype[joint] == 11) {
        offs = offs+1;
    }
    pinout[0] = pin[offs][0];
    pinout[1] = pin[offs][1];
    pinout[2] = pin[offs][2];
}

b1ssgetpres(joint,axis,presout)
    int joint,axis,*presout;
{

    if (b1sschkjaxis(45,joint,axis) != 0) {
        return;
    }
    if (pres[b1ssindx(joint,axis)]  !=  0.) {
        *presout = 1;
    } else {
        *presout = 0;
    }
}

b1ssgetstab(velout,posout)
    double *velout,*posout;
{

    *velout = stabvel;
    *posout = stabpos;
}

b1ssinfo(info)
    int info[50];
{

    info[0] = ground;
    info[1] = nbod;
    info[2] = ndof;
    info[3] = ncons;
    info[4] = nloop;
    info[5] = nldof;
    info[6] = nloopc;
    info[7] = nball;
    info[8] = nlball;
    info[9] = npres;
    info[10] = nuser;
    info[11] = 0;
/* info entries from 12-49 are reserved */
}

b1ssjnt(joint,info,tran)
    int joint,info[50],tran[6];
{
    int i,offs;

    if (b1sschkjnum(48,joint) != 0) {
        return;
    }
    info[0] = jtype[joint];
    info[1] = 0;
    offs = 0;
    info[2] = inb[joint];
    info[3] = outb[joint];
    info[4] = njntdof[joint];
    info[5] = njntc[joint];
    info[6] = njntp[joint];
    info[7] = firstq[joint];
    info[8] = ballq[joint];
    info[9] = firstm[joint];
    info[10] = firstp[joint];
/* info entries from 11-49 are reserved */

    for (i = 0; i <= 5; i++) {
        if (i  <  njntdof[joint]) {
            tran[i] = trans[offs+firstq[joint]+i];
        } else {
            tran[i] = -1;
        }
    }
}

b1sscons(consno,info)
    int consno,info[50];
{

    if (b1sschkucnum(49,consno) != 0) {
        return;
    }
/* There are no user constraints in this problem. */
}
int 
b1sschkbnum(routine,bnum)
    int routine,bnum;
{

    if ((bnum < -1) || (bnum > 4)) {
        b1ssseterr(routine,15);
        return 1;
    }
    return 0;
}
int 
b1sschkjnum(routine,jnum)
    int routine,jnum;
{

    if ((jnum < 0) || (jnum > 4)) {
        b1ssseterr(routine,16);
        return 1;
    }
    return 0;
}
int 
b1sschkucnum(routine,ucnum)
    int routine,ucnum;
{

    if ((ucnum < 0) || (ucnum > -1)) {
        b1ssseterr(routine,21);
        return 1;
    }
    return 0;
}
int 
b1sschkjaxis(routine,jnum,axnum)
    int routine,jnum,axnum;
{
    int maxax;

    if (b1sschkjnum(routine,jnum) != 0) {
        return 1;
    }
    if ((axnum < 0) || (axnum > 6)) {
        b1ssseterr(routine,17);
        return 1;
    }
    maxax = njntdof[jnum]-1;
    if ((jtype[jnum] == 4) || (jtype[jnum] == 6) || (jtype[jnum] == 21)) {
        maxax = maxax+1;
    }
    if (axnum > maxax) {
        b1ssseterr(routine,18);
        return 1;
    }
    return 0;
}
int 
b1sschkjpin(routine,jnum,pinno)
    int routine,jnum,pinno;
{
    int maxax,pinok;

    if (b1sschkjnum(routine,jnum) != 0) {
        return 1;
    }
    if ((pinno < 0) || (pinno > 5)) {
        b1ssseterr(routine,17);
        return 1;
    }
    if (njntdof[jnum] >= 3) {
        maxax = 2;
    } else {
        maxax = njntdof[jnum]-1;
    }
    if (jtype[jnum] == 4) {
        maxax = -1;
    }
    if (jtype[jnum] == 7) {
        maxax = 0;
    }
    pinok = 0;
    if (pinno <= maxax) {
        pinok = 1;
    }
    if (pinok == 0) {
        b1ssseterr(routine,18);
        return 1;
    }
    return 0;
}

/* Convert state to form using 1-2-3 Euler angles for ball joints. */

b1ssst2ang(st,stang)
    double st[5],stang[5];
{
    int i;

    for (i = 0; i < 5; i++) {
        stang[i] = st[i];
    }
}

/* Convert 1-2-3 form of state back to Euler parameters for ball joints. */

b1ssang2st(stang,st)
    double stang[5],st[5];
{
    int i;

    for (i = 0; i < 5; i++) {
        st[i] = stang[i];
    }
}

/* Normalize Euler parameters in state. */

b1ssnrmsterr(st,normst,routine)
    double st[5],normst[5];
    int routine;
{
    int i;

    for (i = 0; i < 5; i++) {
        normst[i] = st[i];
    }
}

b1ssnormst(st,normst)
    double st[5],normst[5];
{

    b1ssnrmsterr(st,normst,0);
}

b1ssgentime(gentm)
    int *gentm;
{

    *gentm = 103328;
}
/*
Done. CPU seconds used: 0.46  Memory used: 942080 bytes.
Equation complexity:
  sdstate:    80 adds   125 multiplies     2 divides   218 assignments
  sdderiv:   281 adds   285 multiplies     5 divides   386 assignments
  sdresid:   191 adds   151 multiplies     0 divides   222 assignments
  sdreac:     98 adds    67 multiplies     0 divides    96 assignments
  sdmom:      38 adds    43 multiplies     0 divides    22 assignments
  sdsys:      57 adds    78 multiplies     0 divides    16 assignments
*/
