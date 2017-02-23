/*
Generated 21-Sep-2004 10:32:03 by SD/FAST, Kane's formulation
(sdfast B.2.6 #70405) on machine ID 690c54f5
Copyright (c) 1990-1996 Symbolic Dynamics, Inc.
Copyright (c) 1990-1996 Parametric Technology Corp.
RESTRICTED RIGHTS LEGEND: Use, duplication, or disclosure by the U.S.
Government is subject to restrictions as set forth in subparagraph
(c)(1)(ii) of the Rights in Technical Data and Computer Software
clause at DFARS 52.227-7013 and similar clauses in the FAR and NASA
FAR Supplement.  Symbolic Dynamics, Inc., Mountain View, CA 94041


ROADMAP (b1c.sd)

Bodies        Inb
No  Name      body Joint type  Coords q         Multipliers
--- --------- ---- ----------- ---------------- -----------------------
 -1 $ground                                    |
  0 pelvis     -1  Planar        0   1   2     |
  1 l.thigh     0  Pin           3             |
  2 l.shin      1  Pin           4             |
  3 r.thigh     0  Pin           5             |
  4 r.shin      3  Pin           6             |

User Constraints

  0 user_0                                     |  0
  1 user_1                                     |  1
  2 user_2                                     |  2
  3 user_3                                     |  3

*/
#include <math.h>
#include <stdio.h>

typedef struct {
    int ground_,nbod_,ndof_,ncons_,nloop_,nldof_,nloopc_,nball_,nlball_,npres_,
      nuser_;
    int jtype_[5],inb_[5],outb_[5],njntdof_[5],njntc_[5],njntp_[5],firstq_[5],
      ballq_[5],firstm_[5],firstp_[5];
    int trans_[7],firstu_[4];
} b1cgtopo_t;
#define ground (b1cgtopo.ground_)
#define nbod (b1cgtopo.nbod_)
#define ndof (b1cgtopo.ndof_)
#define ncons (b1cgtopo.ncons_)
#define nloop (b1cgtopo.nloop_)
#define nldof (b1cgtopo.nldof_)
#define nloopc (b1cgtopo.nloopc_)
#define nball (b1cgtopo.nball_)
#define nlball (b1cgtopo.nlball_)
#define npres (b1cgtopo.npres_)
#define nuser (b1cgtopo.nuser_)
#define jtype (b1cgtopo.jtype_)
#define inb (b1cgtopo.inb_)
#define outb (b1cgtopo.outb_)
#define njntdof (b1cgtopo.njntdof_)
#define njntc (b1cgtopo.njntc_)
#define njntp (b1cgtopo.njntp_)
#define firstq (b1cgtopo.firstq_)
#define ballq (b1cgtopo.ballq_)
#define firstm (b1cgtopo.firstm_)
#define firstp (b1cgtopo.firstp_)
#define trans (b1cgtopo.trans_)
#define firstu (b1cgtopo.firstu_)

typedef struct {
    double grav_[3],mk_[5],ik_[5][3][3],pin_[7][3];
    double rk_[5][3],ri_[5][3],pres_[7],stabvel_,stabpos_;
    int mfrcflg_,roustate_,vpkflg_,inerflg_,mmflg_,mmlduflg_,wwflg_,ltauflg_,
      fs0flg_,ii_,mmap_[7];
    int gravq_[3],mkq_[5],ikq_[5][3][3],pinq_[7][3],rkq_[5][3],riq_[5][3],presq_
      [7],stabvelq_,stabposq_;
    double mtot_,psmkg_,rhead_[5][3],rcom_[5][3],mkrcomt_[5][3][3],psikg_[3][3],
      psrcomg_[3],psrkg_[3],psrig_[3],psmk_[7],psik_[7][3][3],psrcom_[7][3],
      psrk_[7][3],psri_[7][3];
} b1cginput_t;
#define grav (b1cginput.grav_)
#define mk (b1cginput.mk_)
#define ik (b1cginput.ik_)
#define pin (b1cginput.pin_)
#define rk (b1cginput.rk_)
#define ri (b1cginput.ri_)
#define pres (b1cginput.pres_)
#define stabvel (b1cginput.stabvel_)
#define stabpos (b1cginput.stabpos_)
#define rhead (b1cginput.rhead_)
#define rcom (b1cginput.rcom_)
#define psrcomg (b1cginput.psrcomg_)
#define psrcom (b1cginput.psrcom_)
#define mkrcomt (b1cginput.mkrcomt_)
#define psmk (b1cginput.psmk_)
#define psik (b1cginput.psik_)
#define psrk (b1cginput.psrk_)
#define psri (b1cginput.psri_)
#define psmkg (b1cginput.psmkg_)
#define psikg (b1cginput.psikg_)
#define psrkg (b1cginput.psrkg_)
#define psrig (b1cginput.psrig_)
#define mtot (b1cginput.mtot_)
#define mfrcflg (b1cginput.mfrcflg_)
#define roustate (b1cginput.roustate_)
#define vpkflg (b1cginput.vpkflg_)
#define inerflg (b1cginput.inerflg_)
#define mmflg (b1cginput.mmflg_)
#define mmlduflg (b1cginput.mmlduflg_)
#define wwflg (b1cginput.wwflg_)
#define ltauflg (b1cginput.ltauflg_)
#define fs0flg (b1cginput.fs0flg_)
#define ii (b1cginput.ii_)
#define mmap (b1cginput.mmap_)
#define gravq (b1cginput.gravq_)
#define mkq (b1cginput.mkq_)
#define ikq (b1cginput.ikq_)
#define pinq (b1cginput.pinq_)
#define rkq (b1cginput.rkq_)
#define riq (b1cginput.riq_)
#define presq (b1cginput.presq_)
#define stabvelq (b1cginput.stabvelq_)
#define stabposq (b1cginput.stabposq_)

typedef struct {
    double curtim_,q_[7],qn_[7],u_[7],cnk_[7][3][3],cnb_[5][3][3];
    double rnk_[7][3],vnk_[7][3],wk_[7][3],rnb_[5][3],vnb_[5][3],wb_[5][3],
      wbrcom_[5][3],com_[3],rnkg_[3];
    double Cik_[7][3][3],rikt_[7][3][3],Iko_[7][3][3],mkrk_[7][3][3],Cib_[5][3][
      3];
    double Wkk_[7][3],Vkk_[7][3],dik_[7][3],rpp_[7][3],rpk_[7][3],rik_[7][3],
      rik2_[7][3];
    double rpri_[7][3],Wik_[7][3],Vik_[7][3],Wirk_[7][3],rkWkk_[7][3],Wkrpk_[7][
      3],VikWkr_[7][3];
    double perr_[4],verr_[4],aerr_[4],mult_[4],ufk_[5][3],utk_[5][3],mfk_[5][3],
      mtk_[5][3];
    double utau_[7],mtau_[7],uacc_[7],uvel_[7],upos_[7];
    double s2_,c2_,s3_,c3_,s4_,c4_,s5_,c5_,s6_,c6_;
} b1cgstate_t;
#define curtim (b1cgstate.curtim_)
#define q (b1cgstate.q_)
#define qn (b1cgstate.qn_)
#define u (b1cgstate.u_)
#define cnk (b1cgstate.cnk_)
#define cnb (b1cgstate.cnb_)
#define rnkg (b1cgstate.rnkg_)
#define rnk (b1cgstate.rnk_)
#define rnb (b1cgstate.rnb_)
#define vnk (b1cgstate.vnk_)
#define vnb (b1cgstate.vnb_)
#define wk (b1cgstate.wk_)
#define wb (b1cgstate.wb_)
#define com (b1cgstate.com_)
#define Cik (b1cgstate.Cik_)
#define Cib (b1cgstate.Cib_)
#define rikt (b1cgstate.rikt_)
#define Iko (b1cgstate.Iko_)
#define mkrk (b1cgstate.mkrk_)
#define Wkk (b1cgstate.Wkk_)
#define Vkk (b1cgstate.Vkk_)
#define dik (b1cgstate.dik_)
#define rpp (b1cgstate.rpp_)
#define rpk (b1cgstate.rpk_)
#define rik (b1cgstate.rik_)
#define rik2 (b1cgstate.rik2_)
#define rpri (b1cgstate.rpri_)
#define Wik (b1cgstate.Wik_)
#define Vik (b1cgstate.Vik_)
#define Wirk (b1cgstate.Wirk_)
#define rkWkk (b1cgstate.rkWkk_)
#define Wkrpk (b1cgstate.Wkrpk_)
#define VikWkr (b1cgstate.VikWkr_)
#define wbrcom (b1cgstate.wbrcom_)
#define perr (b1cgstate.perr_)
#define verr (b1cgstate.verr_)
#define aerr (b1cgstate.aerr_)
#define mult (b1cgstate.mult_)
#define ufk (b1cgstate.ufk_)
#define utk (b1cgstate.utk_)
#define utau (b1cgstate.utau_)
#define mfk (b1cgstate.mfk_)
#define mtk (b1cgstate.mtk_)
#define mtau (b1cgstate.mtau_)
#define uacc (b1cgstate.uacc_)
#define uvel (b1cgstate.uvel_)
#define upos (b1cgstate.upos_)
#define s2 (b1cgstate.s2_)
#define c2 (b1cgstate.c2_)
#define s3 (b1cgstate.s3_)
#define c3 (b1cgstate.c3_)
#define s4 (b1cgstate.s4_)
#define c4 (b1cgstate.c4_)
#define s5 (b1cgstate.s5_)
#define c5 (b1cgstate.c5_)
#define s6 (b1cgstate.s6_)
#define c6 (b1cgstate.c6_)

typedef struct {
    double fs0_[7],qdot_[7],Otk_[7][3],Atk_[7][3],AiOiWi_[7][3],Fstar_[7][3];
    double Tstar_[7][3],Fstark_[7][3],Tstark_[7][3],IkWk_[7][3],WkIkWk_[7][3],
      gk_[7][3],IkbWk_[5][3],WkIkbWk_[5][3];
    double w0w0_[5],w1w1_[5],w2w2_[5],w0w1_[5],w0w2_[5],w1w2_[5];
    double w00w11_[5],w00w22_[5],w11w22_[5],ww_[4][4],qraux_[4];
    double mm_[7][7],mlo_[7][7],mdi_[7],IkWpk_[7][7][3],works_[7],workss_[7][7];
    double Wpk_[7][7][3],Vpk_[7][7][3],VWri_[7][7][3];
    int wmap_[4],multmap_[4],jpvt_[4],wsiz_,wrank_;
} b1cglhs_t;
#define qdot (b1cglhs.qdot_)
#define Otk (b1cglhs.Otk_)
#define Atk (b1cglhs.Atk_)
#define AiOiWi (b1cglhs.AiOiWi_)
#define Fstar (b1cglhs.Fstar_)
#define Tstar (b1cglhs.Tstar_)
#define fs0 (b1cglhs.fs0_)
#define Fstark (b1cglhs.Fstark_)
#define Tstark (b1cglhs.Tstark_)
#define IkWk (b1cglhs.IkWk_)
#define IkbWk (b1cglhs.IkbWk_)
#define WkIkWk (b1cglhs.WkIkWk_)
#define WkIkbWk (b1cglhs.WkIkbWk_)
#define gk (b1cglhs.gk_)
#define w0w0 (b1cglhs.w0w0_)
#define w1w1 (b1cglhs.w1w1_)
#define w2w2 (b1cglhs.w2w2_)
#define w0w1 (b1cglhs.w0w1_)
#define w0w2 (b1cglhs.w0w2_)
#define w1w2 (b1cglhs.w1w2_)
#define w00w11 (b1cglhs.w00w11_)
#define w00w22 (b1cglhs.w00w22_)
#define w11w22 (b1cglhs.w11w22_)
#define ww (b1cglhs.ww_)
#define qraux (b1cglhs.qraux_)
#define mm (b1cglhs.mm_)
#define mlo (b1cglhs.mlo_)
#define mdi (b1cglhs.mdi_)
#define IkWpk (b1cglhs.IkWpk_)
#define works (b1cglhs.works_)
#define workss (b1cglhs.workss_)
#define Wpk (b1cglhs.Wpk_)
#define Vpk (b1cglhs.Vpk_)
#define VWri (b1cglhs.VWri_)
#define wmap (b1cglhs.wmap_)
#define multmap (b1cglhs.multmap_)
#define jpvt (b1cglhs.jpvt_)
#define wsiz (b1cglhs.wsiz_)
#define wrank (b1cglhs.wrank_)

typedef struct {
    double fs_[7],udot_[7],tauc_[7],dyad_[5][3][3],fc_[7][3],tc_[7][3];
    double ank_[7][3],onk_[7][3],Onkb_[7][3],AOnkri_[7][3],Ankb_[7][3],AnkAtk_[7
      ][3],anb_[5][3],onb_[5][3],dyrcom_[5][3];
    double ffk_[7][3],ttk_[7][3],fccikt_[7][3],ffkb_[5][3],ttkb_[5][3];
} b1cgrhs_t;
#define fs (b1cgrhs.fs_)
#define udot (b1cgrhs.udot_)
#define ank (b1cgrhs.ank_)
#define anb (b1cgrhs.anb_)
#define onk (b1cgrhs.onk_)
#define onb (b1cgrhs.onb_)
#define Onkb (b1cgrhs.Onkb_)
#define AOnkri (b1cgrhs.AOnkri_)
#define Ankb (b1cgrhs.Ankb_)
#define AnkAtk (b1cgrhs.AnkAtk_)
#define dyrcom (b1cgrhs.dyrcom_)
#define ffk (b1cgrhs.ffk_)
#define ttk (b1cgrhs.ttk_)
#define fccikt (b1cgrhs.fccikt_)
#define ffkb (b1cgrhs.ffkb_)
#define ttkb (b1cgrhs.ttkb_)
#define dyad (b1cgrhs.dyad_)
#define fc (b1cgrhs.fc_)
#define tc (b1cgrhs.tc_)
#define tauc (b1cgrhs.tauc_)

typedef struct {
    double temp_[3000],tmat1_[3][3],tmat2_[3][3],tvec1_[3],tvec2_[3],tvec3_[3],
      tvec4_[3],tvec5_[3];
    double tsc1_,tsc2_,tsc3_;
} b1cgtemp_t;
#define temp (b1cgtemp.temp_)
#define tmat1 (b1cgtemp.tmat1_)
#define tmat2 (b1cgtemp.tmat2_)
#define tvec1 (b1cgtemp.tvec1_)
#define tvec2 (b1cgtemp.tvec2_)
#define tvec3 (b1cgtemp.tvec3_)
#define tvec4 (b1cgtemp.tvec4_)
#define tvec5 (b1cgtemp.tvec5_)
#define tsc1 (b1cgtemp.tsc1_)
#define tsc2 (b1cgtemp.tsc2_)
#define tsc3 (b1cgtemp.tsc3_)

b1cgtopo_t b1cgtopo = {
/*  Topological information
*/
    /* ground */ 1,
    /* nbod */ 5,
    /* ndof */ 7,
    /* ncons */ 4,
    /* nloop */ 0,
    /* nldof */ 0,
    /* nloopc */ 0,
    /* nball */ 0,
    /* nlball */ 0,
    /* npres */ 0,
    /* nuser */ 4,
    /* jtype[0] */ 8,
    /* jtype[1] */ 1,
    /* jtype[2] */ 1,
    /* jtype[3] */ 1,
    /* jtype[4] */ 1,
    /* inb[0] */ -1,
    /* inb[1] */ 0,
    /* inb[2] */ 1,
    /* inb[3] */ 0,
    /* inb[4] */ 3,
    /* outb[0] */ 0,
    /* outb[1] */ 1,
    /* outb[2] */ 2,
    /* outb[3] */ 3,
    /* outb[4] */ 4,
    /* njntdof[0] */ 3,
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
    /* firstq[1] */ 3,
    /* firstq[2] */ 4,
    /* firstq[3] */ 5,
    /* firstq[4] */ 6,
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
    /* trans[0] */ 1,
    /* trans[1] */ 1,
    /* trans[2] */ 0,
    /* trans[3] */ 0,
    /* trans[4] */ 0,
    /* trans[5] */ 0,
    /* trans[6] */ 0,
    /* firstu[0] */ 0,
    /* firstu[1] */ 1,
    /* firstu[2] */ 2,
    /* firstu[3] */ 3,
};
b1cginput_t b1cginput = {
/* Model parameters from the input file */

/* gravity */
    /* grav[0] */ 0.,
    /* grav[1] */ 0.,
    /* grav[2] */ -9.81,

/* mass */
    /* mk[0] */ 1.54,
    /* mk[1] */ .54,
    /* mk[2] */ .1,
    /* mk[3] */ .54,
    /* mk[4] */ .1,

/* inertia */
    /* ik[0][0][0] */ 1.,
    /* ik[0][0][1] */ 0.,
    /* ik[0][0][2] */ 0.,
    /* ik[0][1][0] */ 0.,
    /* ik[0][1][1] */ .000645073978666667,
    /* ik[0][1][2] */ 0.,
    /* ik[0][2][0] */ 0.,
    /* ik[0][2][1] */ 0.,
    /* ik[0][2][2] */ 1.,
    /* ik[1][0][0] */ 1.,
    /* ik[1][0][1] */ 0.,
    /* ik[1][0][2] */ 0.,
    /* ik[1][1][0] */ 0.,
    /* ik[1][1][1] */ .000688684713466528,
    /* ik[1][1][2] */ 0.,
    /* ik[1][2][0] */ 0.,
    /* ik[1][2][1] */ 0.,
    /* ik[1][2][2] */ 1.,
    /* ik[2][0][0] */ 1.,
    /* ik[2][0][1] */ 0.,
    /* ik[2][0][2] */ 0.,
    /* ik[2][1][0] */ 0.,
    /* ik[2][1][1] */ .000349461666666667,
    /* ik[2][1][2] */ 0.,
    /* ik[2][2][0] */ 0.,
    /* ik[2][2][1] */ 0.,
    /* ik[2][2][2] */ 1.,
    /* ik[3][0][0] */ 1.,
    /* ik[3][0][1] */ 0.,
    /* ik[3][0][2] */ 0.,
    /* ik[3][1][0] */ 0.,
    /* ik[3][1][1] */ .000688684713466528,
    /* ik[3][1][2] */ 0.,
    /* ik[3][2][0] */ 0.,
    /* ik[3][2][1] */ 0.,
    /* ik[3][2][2] */ 1.,
    /* ik[4][0][0] */ 1.,
    /* ik[4][0][1] */ 0.,
    /* ik[4][0][2] */ 0.,
    /* ik[4][1][0] */ 0.,
    /* ik[4][1][1] */ .000349461666666667,
    /* ik[4][1][2] */ 0.,
    /* ik[4][2][0] */ 0.,
    /* ik[4][2][1] */ 0.,
    /* ik[4][2][2] */ 1.,

/* tree hinge axis vectors */
    /* pin[0][0] */ 1.,
    /* pin[0][1] */ 0.,
    /* pin[0][2] */ 0.,
    /* pin[1][0] */ 0.,
    /* pin[1][1] */ 0.,
    /* pin[1][2] */ 1.,
    /* pin[2][0] */ 0.,
    /* pin[2][1] */ 1.,
    /* pin[2][2] */ 0.,
    /* pin[3][0] */ 0.,
    /* pin[3][1] */ 1.,
    /* pin[3][2] */ 0.,
    /* pin[4][0] */ 0.,
    /* pin[4][1] */ 1.,
    /* pin[4][2] */ 0.,
    /* pin[5][0] */ 0.,
    /* pin[5][1] */ 1.,
    /* pin[5][2] */ 0.,
    /* pin[6][0] */ 0.,
    /* pin[6][1] */ 1.,
    /* pin[6][2] */ 0.,

/* tree bodytojoint vectors */
    /* rk[0][0] */ 0.,
    /* rk[0][1] */ 0.,
    /* rk[0][2] */ 0.,
    /* rk[1][0] */ 0.,
    /* rk[1][1] */ 0.,
    /* rk[1][2] */ .0719666666666667,
    /* rk[2][0] */ 0.,
    /* rk[2][1] */ 0.,
    /* rk[2][2] */ .1016,
    /* rk[3][0] */ 0.,
    /* rk[3][1] */ 0.,
    /* rk[3][2] */ .0719666666666667,
    /* rk[4][0] */ 0.,
    /* rk[4][1] */ 0.,
    /* rk[4][2] */ .1016,

/* tree inbtojoint vectors */
    /* ri[0][0] */ 0.,
    /* ri[0][1] */ 0.,
    /* ri[0][2] */ 0.,
    /* ri[1][0] */ 0.,
    /* ri[1][1] */ 0.,
    /* ri[1][2] */ 0.,
    /* ri[2][0] */ 0.,
    /* ri[2][1] */ 0.,
    /* ri[2][2] */ -.131233333333333,
    /* ri[3][0] */ 0.,
    /* ri[3][1] */ 0.,
    /* ri[3][2] */ 0.,
    /* ri[4][0] */ 0.,
    /* ri[4][1] */ 0.,
    /* ri[4][2] */ -.131233333333333,

/* tree prescribed motion */
    /* pres[0] */ 0.,
    /* pres[1] */ 0.,
    /* pres[2] */ 0.,
    /* pres[3] */ 0.,
    /* pres[4] */ 0.,
    /* pres[5] */ 0.,
    /* pres[6] */ 0.,

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
    /* mmap[5] */ 5,
    /* mmap[6] */ 6,

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
    /* ikq[0][1][1] */ 3,
    /* ikq[0][1][2] */ 0,
    /* ikq[0][2][0] */ 0,
    /* ikq[0][2][1] */ 0,
    /* ikq[0][2][2] */ 0,
    /* ikq[1][0][0] */ 0,
    /* ikq[1][0][1] */ 0,
    /* ikq[1][0][2] */ 0,
    /* ikq[1][1][0] */ 0,
    /* ikq[1][1][1] */ 3,
    /* ikq[1][1][2] */ 0,
    /* ikq[1][2][0] */ 0,
    /* ikq[1][2][1] */ 0,
    /* ikq[1][2][2] */ 0,
    /* ikq[2][0][0] */ 0,
    /* ikq[2][0][1] */ 0,
    /* ikq[2][0][2] */ 0,
    /* ikq[2][1][0] */ 0,
    /* ikq[2][1][1] */ 3,
    /* ikq[2][1][2] */ 0,
    /* ikq[2][2][0] */ 0,
    /* ikq[2][2][1] */ 0,
    /* ikq[2][2][2] */ 0,
    /* ikq[3][0][0] */ 0,
    /* ikq[3][0][1] */ 0,
    /* ikq[3][0][2] */ 0,
    /* ikq[3][1][0] */ 0,
    /* ikq[3][1][1] */ 3,
    /* ikq[3][1][2] */ 0,
    /* ikq[3][2][0] */ 0,
    /* ikq[3][2][1] */ 0,
    /* ikq[3][2][2] */ 0,
    /* ikq[4][0][0] */ 0,
    /* ikq[4][0][1] */ 0,
    /* ikq[4][0][2] */ 0,
    /* ikq[4][1][0] */ 0,
    /* ikq[4][1][1] */ 3,
    /* ikq[4][1][2] */ 0,
    /* ikq[4][2][0] */ 0,
    /* ikq[4][2][1] */ 0,
    /* ikq[4][2][2] */ 0,
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
    /* pinq[5][0] */ 0,
    /* pinq[5][1] */ 0,
    /* pinq[5][2] */ 0,
    /* pinq[6][0] */ 0,
    /* pinq[6][1] */ 0,
    /* pinq[6][2] */ 0,
    /* rkq[0][0] */ 0,
    /* rkq[0][1] */ 0,
    /* rkq[0][2] */ 3,
    /* rkq[1][0] */ 0,
    /* rkq[1][1] */ 0,
    /* rkq[1][2] */ 3,
    /* rkq[2][0] */ 0,
    /* rkq[2][1] */ 0,
    /* rkq[2][2] */ 3,
    /* rkq[3][0] */ 0,
    /* rkq[3][1] */ 0,
    /* rkq[3][2] */ 3,
    /* rkq[4][0] */ 0,
    /* rkq[4][1] */ 0,
    /* rkq[4][2] */ 3,
    /* riq[0][0] */ 0,
    /* riq[0][1] */ 0,
    /* riq[0][2] */ 0,
    /* riq[1][0] */ 0,
    /* riq[1][1] */ 0,
    /* riq[1][2] */ 3,
    /* riq[2][0] */ 0,
    /* riq[2][1] */ 0,
    /* riq[2][2] */ 3,
    /* riq[3][0] */ 0,
    /* riq[3][1] */ 0,
    /* riq[3][2] */ 3,
    /* riq[4][0] */ 0,
    /* riq[4][1] */ 0,
    /* riq[4][2] */ 3,
    /* presq[0] */ 0,
    /* presq[1] */ 0,
    /* presq[2] */ 0,
    /* presq[3] */ 0,
    /* presq[4] */ 0,
    /* presq[5] */ 0,
    /* presq[6] */ 0,
    /* stabvelq */ 3,
    /* stabposq */ 3,

/* End of values from input file */

};
b1cgstate_t b1cgstate;
b1cglhs_t b1cglhs;
b1cgrhs_t b1cgrhs;
b1cgtemp_t b1cgtemp;


b1cinit()
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
            b1cseterr(7,25);
        }
    }
    for (k = 0; k < 5; k++) {
        if (mkq[k] == 1) {
            b1cseterr(7,26);
        }
        for (i = 0; i < 3; i++) {
            if (rkq[k][i] == 1) {
                b1cseterr(7,29);
            }
            if (riq[k][i] == 1) {
                b1cseterr(7,30);
            }
            for (j = 0; j < 3; j++) {
                if (ikq[k][i][j] == 1) {
                    b1cseterr(7,27);
                }
            }
        }
    }
    for (k = 0; k < 7; k++) {
        for (i = 0; i < 3; i++) {
            if (pinq[k][i] == 1) {
                b1cseterr(7,28);
            }
        }
    }

/* Normalize pin vectors if necessary */


/* Zero out Vpk and Wpk */

    for (i = 0; i < 7; i++) {
        for (j = i; j <= 6; j++) {
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
    dik[3][2] = (ri[1][2]-rk[0][2]);
    dik[4][2] = (ri[2][2]-rk[1][2]);
    dik[5][2] = (ri[3][2]-rk[0][2]);
    dik[6][2] = (ri[4][2]-rk[3][2]);

/* Compute mass properties-related constants */

    mtot = (mk[4]+(mk[3]+(mk[2]+(mk[0]+mk[1]))));
    mkrk[2][0][1] = -(mk[0]*rk[0][2]);
    mkrk[2][1][0] = (mk[0]*rk[0][2]);
    mkrk[3][0][1] = -(mk[1]*rk[1][2]);
    mkrk[3][1][0] = (mk[1]*rk[1][2]);
    mkrk[4][0][1] = -(mk[2]*rk[2][2]);
    mkrk[4][1][0] = (mk[2]*rk[2][2]);
    mkrk[5][0][1] = -(mk[3]*rk[3][2]);
    mkrk[5][1][0] = (mk[3]*rk[3][2]);
    mkrk[6][0][1] = -(mk[4]*rk[4][2]);
    mkrk[6][1][0] = (mk[4]*rk[4][2]);
    Iko[2][0][0] = (1.-(mkrk[2][0][1]*rk[0][2]));
    Iko[2][1][1] = (ik[0][1][1]+(mkrk[2][1][0]*rk[0][2]));
    Iko[3][0][0] = (1.-(mkrk[3][0][1]*rk[1][2]));
    Iko[3][1][1] = (ik[1][1][1]+(mkrk[3][1][0]*rk[1][2]));
    Iko[4][0][0] = (1.-(mkrk[4][0][1]*rk[2][2]));
    Iko[4][1][1] = (ik[2][1][1]+(mkrk[4][1][0]*rk[2][2]));
    Iko[5][0][0] = (1.-(mkrk[5][0][1]*rk[3][2]));
    Iko[5][1][1] = (ik[3][1][1]+(mkrk[5][1][0]*rk[3][2]));
    Iko[6][0][0] = (1.-(mkrk[6][0][1]*rk[4][2]));
    Iko[6][1][1] = (ik[4][1][1]+(mkrk[6][1][0]*rk[4][2]));
    b1cserialno(&i);
    if (i != 70405) {
        b1cseterr(7,41);
    }
    roustate = 1;
}

b1cstate(timein,qin,uin)
    double timein,qin[7],uin[7];
{
/*
Compute kinematic information and store it in sdgstate.

Generated 21-Sep-2004 10:32:03 by SD/FAST, Kane's formulation
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
        b1cseterr(8,22);
        return;
    }
    if (roustate == 1) {
        for (i = 0; i < 7; i++) {
            if (presq[i] == 1) {
                b1cseterr(8,31);
            }
        }
    }
/*
See if time or any qs have changed since last call
*/
    if ((roustate != 1) && (timein == curtim)) {
        qchg = 0;
        for (i = 0; i < 7; i++) {
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
        for (i = 0; i < 7; i++) {
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
    for (i = 0; i < 7; i++) {
        q[i] = qin[i];
    }
/*
Compute sines and cosines of q
*/
    s2 = sin(q[2]);
    c2 = cos(q[2]);
    s3 = sin(q[3]);
    c3 = cos(q[3]);
    s4 = sin(q[4]);
    c4 = cos(q[4]);
    s5 = sin(q[5]);
    c5 = cos(q[5]);
    s6 = sin(q[6]);
    c6 = cos(q[6]);
/*
Compute across-axis direction cosines Cik
*/
/*
Compute across-joint direction cosines Cib
*/
/*
Compute gravity
*/
    gk[3][0] = (9.81*((s2*c3)+(s3*c2)));
    gk[3][2] = (9.81*((s2*s3)-(c2*c3)));
    gk[4][0] = ((gk[3][0]*c4)-(gk[3][2]*s4));
    gk[4][2] = ((gk[3][0]*s4)+(gk[3][2]*c4));
    gk[5][0] = (9.81*((s2*c5)+(s5*c2)));
    gk[5][2] = (9.81*((s2*s5)-(c2*c5)));
    gk[6][0] = ((gk[5][0]*c6)-(gk[5][2]*s6));
    gk[6][2] = ((gk[5][0]*s6)+(gk[5][2]*c6));
/*
Compute cnk & cnb (direction cosines in N)
*/
    cnk[3][0][0] = ((c2*c3)-(s2*s3));
    cnk[3][0][2] = ((s2*c3)+(s3*c2));
    cnk[3][2][0] = -((s2*c3)+(s3*c2));
    cnk[3][2][2] = ((c2*c3)-(s2*s3));
    cnk[4][0][0] = ((cnk[3][0][0]*c4)-(cnk[3][0][2]*s4));
    cnk[4][0][2] = ((cnk[3][0][0]*s4)+(cnk[3][0][2]*c4));
    cnk[4][2][0] = ((cnk[3][2][0]*c4)-(cnk[3][2][2]*s4));
    cnk[4][2][2] = ((cnk[3][2][0]*s4)+(cnk[3][2][2]*c4));
    cnk[5][0][0] = ((c2*c5)-(s2*s5));
    cnk[5][0][2] = ((s2*c5)+(s5*c2));
    cnk[5][2][0] = -((s2*c5)+(s5*c2));
    cnk[5][2][2] = ((c2*c5)-(s2*s5));
    cnk[6][0][0] = ((cnk[5][0][0]*c6)-(cnk[5][0][2]*s6));
    cnk[6][0][2] = ((cnk[5][0][0]*s6)+(cnk[5][0][2]*c6));
    cnk[6][2][0] = ((cnk[5][2][0]*c6)-(cnk[5][2][2]*s6));
    cnk[6][2][2] = ((cnk[5][2][0]*s6)+(cnk[5][2][2]*c6));
    cnb[0][0][0] = c2;
    cnb[0][0][1] = 0.;
    cnb[0][0][2] = s2;
    cnb[0][1][0] = 0.;
    cnb[0][1][1] = 1.;
    cnb[0][1][2] = 0.;
    cnb[0][2][0] = -s2;
    cnb[0][2][1] = 0.;
    cnb[0][2][2] = c2;
    cnb[1][0][0] = cnk[3][0][0];
    cnb[1][0][1] = 0.;
    cnb[1][0][2] = cnk[3][0][2];
    cnb[1][1][0] = 0.;
    cnb[1][1][1] = 1.;
    cnb[1][1][2] = 0.;
    cnb[1][2][0] = cnk[3][2][0];
    cnb[1][2][1] = 0.;
    cnb[1][2][2] = cnk[3][2][2];
    cnb[2][0][0] = cnk[4][0][0];
    cnb[2][0][1] = 0.;
    cnb[2][0][2] = cnk[4][0][2];
    cnb[2][1][0] = 0.;
    cnb[2][1][1] = 1.;
    cnb[2][1][2] = 0.;
    cnb[2][2][0] = cnk[4][2][0];
    cnb[2][2][1] = 0.;
    cnb[2][2][2] = cnk[4][2][2];
    cnb[3][0][0] = cnk[5][0][0];
    cnb[3][0][1] = 0.;
    cnb[3][0][2] = cnk[5][0][2];
    cnb[3][1][0] = 0.;
    cnb[3][1][1] = 1.;
    cnb[3][1][2] = 0.;
    cnb[3][2][0] = cnk[5][2][0];
    cnb[3][2][1] = 0.;
    cnb[3][2][2] = cnk[5][2][2];
    cnb[4][0][0] = cnk[6][0][0];
    cnb[4][0][1] = 0.;
    cnb[4][0][2] = cnk[6][0][2];
    cnb[4][1][0] = 0.;
    cnb[4][1][1] = 1.;
    cnb[4][1][2] = 0.;
    cnb[4][2][0] = cnk[6][2][0];
    cnb[4][2][1] = 0.;
    cnb[4][2][2] = cnk[6][2][2];
/*
Compute q-related auxiliary variables
*/
    rik[3][0] = -(ri[1][2]*s3);
    rik[3][2] = ((ri[1][2]*c3)-rk[1][2]);
    rik[4][0] = -(ri[2][2]*s4);
    rik[4][2] = ((ri[2][2]*c4)-rk[2][2]);
    rik[5][0] = -(ri[3][2]*s5);
    rik[5][2] = ((ri[3][2]*c5)-rk[3][2]);
    rik[6][0] = -(ri[4][2]*s6);
    rik[6][2] = ((ri[4][2]*c6)-rk[4][2]);
/*
Compute rnk & rnb (mass center locations in N)
*/
    rnk[2][0] = (q[0]-(rk[0][2]*s2));
    rnk[2][2] = (q[1]-(rk[0][2]*c2));
    rnk[3][0] = ((rnk[2][0]+(ri[1][2]*s2))-(cnk[3][0][2]*rk[1][2]));
    rnk[3][2] = ((rnk[2][2]+(ri[1][2]*c2))-(cnk[3][2][2]*rk[1][2]));
    rnk[4][0] = ((rnk[3][0]+(cnk[3][0][2]*ri[2][2]))-(cnk[4][0][2]*rk[2][2]));
    rnk[4][2] = ((rnk[3][2]+(cnk[3][2][2]*ri[2][2]))-(cnk[4][2][2]*rk[2][2]));
    rnk[5][0] = ((rnk[2][0]+(ri[3][2]*s2))-(cnk[5][0][2]*rk[3][2]));
    rnk[5][2] = ((rnk[2][2]+(ri[3][2]*c2))-(cnk[5][2][2]*rk[3][2]));
    rnk[6][0] = ((rnk[5][0]+(cnk[5][0][2]*ri[4][2]))-(cnk[6][0][2]*rk[4][2]));
    rnk[6][2] = ((rnk[5][2]+(cnk[5][2][2]*ri[4][2]))-(cnk[6][2][2]*rk[4][2]));
    rnb[0][0] = rnk[2][0];
    rnb[0][1] = 0.;
    rnb[0][2] = rnk[2][2];
    rnb[1][0] = rnk[3][0];
    rnb[1][1] = 0.;
    rnb[1][2] = rnk[3][2];
    rnb[2][0] = rnk[4][0];
    rnb[2][1] = 0.;
    rnb[2][2] = rnk[4][2];
    rnb[3][0] = rnk[5][0];
    rnb[3][1] = 0.;
    rnb[3][2] = rnk[5][2];
    rnb[4][0] = rnk[6][0];
    rnb[4][1] = 0.;
    rnb[4][2] = rnk[6][2];
/*
Compute com (system mass center location in N)
*/
    com[0] = ((1./mtot)*((mk[4]*rnk[6][0])+((mk[3]*rnk[5][0])+((mk[2]*rnk[4][0])
      +((mk[0]*rnk[2][0])+(mk[1]*rnk[3][0]))))));
    com[1] = 0.;
    com[2] = ((1./mtot)*((mk[4]*rnk[6][2])+((mk[3]*rnk[5][2])+((mk[2]*rnk[4][2])
      +((mk[0]*rnk[2][2])+(mk[1]*rnk[3][2]))))));
/*
Compute constraint position errors
*/
    skipqs: ;
    b1cuperr(curtim,q,&perr[0]);
    if (uchg == 0) {
        goto skipus;
    }
/*
Velocity-related variables need to be computed
*/
    inerflg = 0;
    for (i = 0; i < 7; i++) {
        u[i] = uin[i];
    }
/*
Compute u-related auxiliary variables
*/
/*
Compute wk & wb (angular velocities)
*/
    wk[3][1] = (u[2]+u[3]);
    wk[4][1] = (u[4]+wk[3][1]);
    wk[5][1] = (u[2]+u[5]);
    wk[6][1] = (u[6]+wk[5][1]);
    wb[0][0] = 0.;
    wb[0][1] = u[2];
    wb[0][2] = 0.;
    wb[1][0] = 0.;
    wb[1][1] = wk[3][1];
    wb[1][2] = 0.;
    wb[2][0] = 0.;
    wb[2][1] = wk[4][1];
    wb[2][2] = 0.;
    wb[3][0] = 0.;
    wb[3][1] = wk[5][1];
    wb[3][2] = 0.;
    wb[4][0] = 0.;
    wb[4][1] = wk[6][1];
    wb[4][2] = 0.;
/*
Compute auxiliary variables involving wk
*/
    Wirk[3][0] = (ri[1][2]*u[2]);
    Wirk[4][0] = (ri[2][2]*wk[3][1]);
    Wirk[5][0] = (ri[3][2]*u[2]);
    Wirk[6][0] = (ri[4][2]*wk[5][1]);
    Wkrpk[2][0] = -(rk[0][2]*u[2]);
    Wkrpk[3][0] = -(rk[1][2]*wk[3][1]);
    Wkrpk[4][0] = -(rk[2][2]*wk[4][1]);
    Wkrpk[5][0] = -(rk[3][2]*wk[5][1]);
    Wkrpk[6][0] = -(rk[4][2]*wk[6][1]);
    IkWk[2][1] = (ik[0][1][1]*u[2]);
    IkWk[3][1] = (ik[1][1][1]*wk[3][1]);
    IkWk[4][1] = (ik[2][1][1]*wk[4][1]);
    IkWk[5][1] = (ik[3][1][1]*wk[5][1]);
    IkWk[6][1] = (ik[4][1][1]*wk[6][1]);
/*
Compute temporaries for use in SDRHS
*/
    w1w1[0] = (u[2]*u[2]);
    w1w1[1] = (wk[3][1]*wk[3][1]);
    w1w1[2] = (wk[4][1]*wk[4][1]);
    w1w1[3] = (wk[5][1]*wk[5][1]);
    w1w1[4] = (wk[6][1]*wk[6][1]);
/*
Compute vnk & vnb (mass center linear velocities in N)
*/
    vnk[2][0] = (u[0]+(Wkrpk[2][0]*c2));
    vnk[2][2] = (u[1]-(Wkrpk[2][0]*s2));
    vnk[3][0] = ((cnk[3][0][0]*Wkrpk[3][0])+(vnk[2][0]+(Wirk[3][0]*c2)));
    vnk[3][2] = ((cnk[3][2][0]*Wkrpk[3][0])+(vnk[2][2]-(Wirk[3][0]*s2)));
    vnk[4][0] = ((cnk[4][0][0]*Wkrpk[4][0])+(vnk[3][0]+(cnk[3][0][0]*Wirk[4][0])
      ));
    vnk[4][2] = ((cnk[4][2][0]*Wkrpk[4][0])+(vnk[3][2]+(cnk[3][2][0]*Wirk[4][0])
      ));
    vnk[5][0] = ((cnk[5][0][0]*Wkrpk[5][0])+(vnk[2][0]+(Wirk[5][0]*c2)));
    vnk[5][2] = ((cnk[5][2][0]*Wkrpk[5][0])+(vnk[2][2]-(Wirk[5][0]*s2)));
    vnk[6][0] = ((cnk[6][0][0]*Wkrpk[6][0])+(vnk[5][0]+(cnk[5][0][0]*Wirk[6][0])
      ));
    vnk[6][2] = ((cnk[6][2][0]*Wkrpk[6][0])+(vnk[5][2]+(cnk[5][2][0]*Wirk[6][0])
      ));
    vnb[0][0] = vnk[2][0];
    vnb[0][1] = 0.;
    vnb[0][2] = vnk[2][2];
    vnb[1][0] = vnk[3][0];
    vnb[1][1] = 0.;
    vnb[1][2] = vnk[3][2];
    vnb[2][0] = vnk[4][0];
    vnb[2][1] = 0.;
    vnb[2][2] = vnk[4][2];
    vnb[3][0] = vnk[5][0];
    vnb[3][1] = 0.;
    vnb[3][2] = vnk[5][2];
    vnb[4][0] = vnk[6][0];
    vnb[4][1] = 0.;
    vnb[4][2] = vnk[6][2];
/*
Compute qdot (kinematical equations)
*/
    qdot[0] = u[0];
    qdot[1] = u[1];
    qdot[2] = u[2];
    qdot[3] = u[3];
    qdot[4] = u[4];
    qdot[5] = u[5];
    qdot[6] = u[6];
/*
Compute constraint velocity errors
*/
    skipus: ;
    b1cuverr(curtim,q,u,&verr[0]);
/*
Initialize applied forces and torques to zero
*/
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 3; j++) {
            ufk[i][j] = 0.;
            utk[i][j] = 0.;
        }
    }
    for (i = 0; i < 7; i++) {
        utau[i] = 0.;
    }
    ltauflg = 0;
    fs0flg = 0;
/*
 Used 0.10 seconds CPU time,
 16384 additional bytes of memory.
 Equations contain   88 adds/subtracts/negates
                    127 multiplies
                      2 divides
                    226 assignments
*/
}

b1cqdot(oqdot)
    double oqdot[7];
{
/*
Return position coordinate derivatives for tree joints.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(63,23);
        return;
    }
    for (i = 0; i <= 6; i++) {
        oqdot[i] = qdot[i];
    }
}

b1cu2qdot(uin,oqdot)
    double uin[7],oqdot[7];
{
/*
Convert velocities to qdots.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(64,23);
        return;
    }
    for (i = 0; i <= 6; i++) {
        oqdot[i] = uin[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      7 assignments
*/
}

b1cpsstate(lqin)
    double lqin[1];
{

    if (roustate != 2) {
        b1cseterr(9,23);
        return;
    }
}

b1cdovpk()
{

    if (vpkflg == 0) {
/*
Compute Wpk (partial angular velocities)
*/
        Wpk[2][2][1] = 1.;
        Wpk[2][3][1] = 1.;
        Wpk[2][4][1] = 1.;
        Wpk[2][5][1] = 1.;
        Wpk[2][6][1] = 1.;
        Wpk[3][3][1] = 1.;
        Wpk[3][4][1] = 1.;
        Wpk[4][4][1] = 1.;
        Wpk[5][5][1] = 1.;
        Wpk[5][6][1] = 1.;
        Wpk[6][6][1] = 1.;
/*
Compute Vpk (partial velocities)
*/
        Vpk[0][0][0] = 1.;
        Vpk[0][1][0] = 1.;
        Vpk[0][2][0] = c2;
        Vpk[0][2][2] = s2;
        Vpk[0][3][0] = ((c2*c3)-(s2*s3));
        Vpk[0][3][2] = ((s2*c3)+(s3*c2));
        Vpk[0][4][0] = ((Vpk[0][3][0]*c4)-(Vpk[0][3][2]*s4));
        Vpk[0][4][2] = ((Vpk[0][3][0]*s4)+(Vpk[0][3][2]*c4));
        Vpk[0][5][0] = ((c2*c5)-(s2*s5));
        Vpk[0][5][2] = ((s2*c5)+(s5*c2));
        Vpk[0][6][0] = ((Vpk[0][5][0]*c6)-(Vpk[0][5][2]*s6));
        Vpk[0][6][2] = ((Vpk[0][5][0]*s6)+(Vpk[0][5][2]*c6));
        Vpk[1][1][2] = 1.;
        Vpk[1][2][0] = -s2;
        Vpk[1][2][2] = c2;
        Vpk[1][3][0] = -((s2*c3)+(s3*c2));
        Vpk[1][3][2] = ((c2*c3)-(s2*s3));
        Vpk[1][4][0] = ((Vpk[1][3][0]*c4)-(Vpk[1][3][2]*s4));
        Vpk[1][4][2] = ((Vpk[1][3][0]*s4)+(Vpk[1][3][2]*c4));
        Vpk[1][5][0] = -((s2*c5)+(s5*c2));
        Vpk[1][5][2] = ((c2*c5)-(s2*s5));
        Vpk[1][6][0] = ((Vpk[1][5][0]*c6)-(Vpk[1][5][2]*s6));
        Vpk[1][6][2] = ((Vpk[1][5][0]*s6)+(Vpk[1][5][2]*c6));
        Vpk[2][2][0] = -rk[0][2];
        VWri[2][3][0] = (ri[1][2]-rk[0][2]);
        Vpk[2][3][0] = ((VWri[2][3][0]*c3)-rk[1][2]);
        Vpk[2][3][2] = (VWri[2][3][0]*s3);
        VWri[2][4][0] = (ri[2][2]+Vpk[2][3][0]);
        Vpk[2][4][0] = (((VWri[2][4][0]*c4)-(Vpk[2][3][2]*s4))-rk[2][2]);
        Vpk[2][4][2] = ((Vpk[2][3][2]*c4)+(VWri[2][4][0]*s4));
        VWri[2][5][0] = (ri[3][2]-rk[0][2]);
        Vpk[2][5][0] = ((VWri[2][5][0]*c5)-rk[3][2]);
        Vpk[2][5][2] = (VWri[2][5][0]*s5);
        VWri[2][6][0] = (ri[4][2]+Vpk[2][5][0]);
        Vpk[2][6][0] = (((VWri[2][6][0]*c6)-(Vpk[2][5][2]*s6))-rk[4][2]);
        Vpk[2][6][2] = ((Vpk[2][5][2]*c6)+(VWri[2][6][0]*s6));
        Vpk[3][3][0] = -rk[1][2];
        VWri[3][4][0] = (ri[2][2]-rk[1][2]);
        Vpk[3][4][0] = ((VWri[3][4][0]*c4)-rk[2][2]);
        Vpk[3][4][2] = (VWri[3][4][0]*s4);
        Vpk[4][4][0] = -rk[2][2];
        Vpk[5][5][0] = -rk[3][2];
        VWri[5][6][0] = (ri[4][2]-rk[3][2]);
        Vpk[5][6][0] = ((VWri[5][6][0]*c6)-rk[4][2]);
        Vpk[5][6][2] = (VWri[5][6][0]*s6);
        Vpk[6][6][0] = -rk[4][2];
        vpkflg = 1;
    }
/*
 Used 0.04 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain   40 adds/subtracts/negates
                     48 multiplies
                      0 divides
                     57 assignments
*/
}

b1cdoltau()
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

b1cdoiner()
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
        Atk[2][2] = -(u[2]*Wkrpk[2][0]);
        AiOiWi[3][2] = (Atk[2][2]-(u[2]*Wirk[3][0]));
        Atk[3][0] = -(AiOiWi[3][2]*s3);
        Atk[3][2] = ((AiOiWi[3][2]*c3)-(wk[3][1]*Wkrpk[3][0]));
        AiOiWi[4][2] = (Atk[3][2]-(Wirk[4][0]*wk[3][1]));
        Atk[4][0] = ((Atk[3][0]*c4)-(AiOiWi[4][2]*s4));
        Atk[4][2] = (((AiOiWi[4][2]*c4)+(Atk[3][0]*s4))-(wk[4][1]*Wkrpk[4][0]));
        AiOiWi[5][2] = (Atk[2][2]-(u[2]*Wirk[5][0]));
        Atk[5][0] = -(AiOiWi[5][2]*s5);
        Atk[5][2] = ((AiOiWi[5][2]*c5)-(wk[5][1]*Wkrpk[5][0]));
        AiOiWi[6][2] = (Atk[5][2]-(Wirk[6][0]*wk[5][1]));
        Atk[6][0] = ((Atk[5][0]*c6)-(AiOiWi[6][2]*s6));
        Atk[6][2] = (((AiOiWi[6][2]*c6)+(Atk[5][0]*s6))-(wk[6][1]*Wkrpk[6][0]));
        inerflg = 1;
    }
/*
 Used 0.02 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   15 adds/subtracts/negates
                     21 multiplies
                      0 divides
                     13 assignments
*/
}

b1cdofs0()
{

/*
Compute effect of all applied loads
*/
    if (fs0flg == 0) {
        b1cdoltau();
        b1cdoiner();
/*
Compute Fstar (forces)
*/
        Fstar[2][0] = -(ufk[0][0]+(9.81*(mk[0]*s2)));
        Fstar[2][2] = ((mk[0]*(Atk[2][2]+(9.81*c2)))-ufk[0][2]);
        Fstar[3][0] = ((mk[1]*(Atk[3][0]-gk[3][0]))-ufk[1][0]);
        Fstar[3][2] = ((mk[1]*(Atk[3][2]-gk[3][2]))-ufk[1][2]);
        Fstar[4][0] = ((mk[2]*(Atk[4][0]-gk[4][0]))-ufk[2][0]);
        Fstar[4][2] = ((mk[2]*(Atk[4][2]-gk[4][2]))-ufk[2][2]);
        Fstar[5][0] = ((mk[3]*(Atk[5][0]-gk[5][0]))-ufk[3][0]);
        Fstar[5][2] = ((mk[3]*(Atk[5][2]-gk[5][2]))-ufk[3][2]);
        Fstar[6][0] = ((mk[4]*(Atk[6][0]-gk[6][0]))-ufk[4][0]);
        Fstar[6][2] = ((mk[4]*(Atk[6][2]-gk[6][2]))-ufk[4][2]);
/*
Compute Tstar (torques)
*/
/*
Compute fs0 (RHS ignoring constraints)
*/
        b1cdovpk();
        fs0[0] = (utau[0]-(((Fstar[6][0]*Vpk[0][6][0])+(Fstar[6][2]*Vpk[0][6][2]
          ))+(((Fstar[5][0]*Vpk[0][5][0])+(Fstar[5][2]*Vpk[0][5][2]))+(((
          Fstar[4][0]*Vpk[0][4][0])+(Fstar[4][2]*Vpk[0][4][2]))+(((Fstar[2][0]*
          c2)+(Fstar[2][2]*s2))+((Fstar[3][0]*Vpk[0][3][0])+(Fstar[3][2]*
          Vpk[0][3][2])))))));
        fs0[1] = (utau[1]-(((Fstar[6][0]*Vpk[1][6][0])+(Fstar[6][2]*Vpk[1][6][2]
          ))+(((Fstar[5][0]*Vpk[1][5][0])+(Fstar[5][2]*Vpk[1][5][2]))+(((
          Fstar[4][0]*Vpk[1][4][0])+(Fstar[4][2]*Vpk[1][4][2]))+(((Fstar[2][2]*
          c2)-(Fstar[2][0]*s2))+((Fstar[3][0]*Vpk[1][3][0])+(Fstar[3][2]*
          Vpk[1][3][2])))))));
        fs0[2] = (utau[2]-((((Fstar[6][0]*Vpk[2][6][0])+(Fstar[6][2]*
          Vpk[2][6][2]))-utk[4][1])+((((Fstar[5][0]*Vpk[2][5][0])+(Fstar[5][2]*
          Vpk[2][5][2]))-utk[3][1])+((((Fstar[4][0]*Vpk[2][4][0])+(Fstar[4][2]*
          Vpk[2][4][2]))-utk[2][1])+((((Fstar[3][0]*Vpk[2][3][0])+(Fstar[3][2]*
          Vpk[2][3][2]))-utk[1][1])-(utk[0][1]+(Fstar[2][0]*rk[0][2])))))));
        fs0[3] = (utau[3]-((((Fstar[4][0]*Vpk[3][4][0])+(Fstar[4][2]*
          Vpk[3][4][2]))-utk[2][1])-(utk[1][1]+(Fstar[3][0]*rk[1][2]))));
        fs0[4] = (utau[4]+(utk[2][1]+(Fstar[4][0]*rk[2][2])));
        fs0[5] = (utau[5]-((((Fstar[6][0]*Vpk[5][6][0])+(Fstar[6][2]*
          Vpk[5][6][2]))-utk[4][1])-(utk[3][1]+(Fstar[5][0]*rk[3][2]))));
        fs0[6] = (utau[6]+(utk[4][1]+(Fstar[6][0]*rk[4][2])));
        fs0flg = 1;
    }
/*
 Used 0.03 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain   68 adds/subtracts/negates
                     49 multiplies
                      0 divides
                     17 assignments
*/
}

b1cdomm(routine)
    int routine;
{
    int i;

    if (mmflg == 0) {
/*
Compute mass matrix (MM)
*/
        b1cdovpk();
        mm[0][0] = ((mk[4]*((Vpk[0][6][0]*Vpk[0][6][0])+(Vpk[0][6][2]*
          Vpk[0][6][2])))+((mk[3]*((Vpk[0][5][0]*Vpk[0][5][0])+(Vpk[0][5][2]*
          Vpk[0][5][2])))+((mk[0]+(mk[1]*((Vpk[0][3][0]*Vpk[0][3][0])+(
          Vpk[0][3][2]*Vpk[0][3][2]))))+(mk[2]*((Vpk[0][4][0]*Vpk[0][4][0])+(
          Vpk[0][4][2]*Vpk[0][4][2]))))));
        mm[0][1] = ((mk[4]*((Vpk[0][6][0]*Vpk[1][6][0])+(Vpk[0][6][2]*
          Vpk[1][6][2])))+((mk[3]*((Vpk[0][5][0]*Vpk[1][5][0])+(Vpk[0][5][2]*
          Vpk[1][5][2])))+((mk[1]*((Vpk[0][3][0]*Vpk[1][3][0])+(Vpk[0][3][2]*
          Vpk[1][3][2])))+(mk[2]*((Vpk[0][4][0]*Vpk[1][4][0])+(Vpk[0][4][2]*
          Vpk[1][4][2]))))));
        mm[0][2] = ((mk[4]*((Vpk[0][6][0]*Vpk[2][6][0])+(Vpk[0][6][2]*
          Vpk[2][6][2])))+((mk[3]*((Vpk[0][5][0]*Vpk[2][5][0])+(Vpk[0][5][2]*
          Vpk[2][5][2])))+((mk[2]*((Vpk[0][4][0]*Vpk[2][4][0])+(Vpk[0][4][2]*
          Vpk[2][4][2])))+((mk[1]*((Vpk[0][3][0]*Vpk[2][3][0])+(Vpk[0][3][2]*
          Vpk[2][3][2])))-(mk[0]*(rk[0][2]*c2))))));
        mm[0][3] = ((mk[2]*((Vpk[0][4][0]*Vpk[3][4][0])+(Vpk[0][4][2]*
          Vpk[3][4][2])))-(mk[1]*(rk[1][2]*Vpk[0][3][0])));
        mm[0][4] = -(mk[2]*(rk[2][2]*Vpk[0][4][0]));
        mm[0][5] = ((mk[4]*((Vpk[0][6][0]*Vpk[5][6][0])+(Vpk[0][6][2]*
          Vpk[5][6][2])))-(mk[3]*(rk[3][2]*Vpk[0][5][0])));
        mm[0][6] = -(mk[4]*(rk[4][2]*Vpk[0][6][0]));
        mm[1][1] = ((mk[4]*((Vpk[1][6][0]*Vpk[1][6][0])+(Vpk[1][6][2]*
          Vpk[1][6][2])))+((mk[3]*((Vpk[1][5][0]*Vpk[1][5][0])+(Vpk[1][5][2]*
          Vpk[1][5][2])))+((mk[0]+(mk[1]*((Vpk[1][3][0]*Vpk[1][3][0])+(
          Vpk[1][3][2]*Vpk[1][3][2]))))+(mk[2]*((Vpk[1][4][0]*Vpk[1][4][0])+(
          Vpk[1][4][2]*Vpk[1][4][2]))))));
        mm[1][2] = ((mk[4]*((Vpk[1][6][0]*Vpk[2][6][0])+(Vpk[1][6][2]*
          Vpk[2][6][2])))+((mk[3]*((Vpk[1][5][0]*Vpk[2][5][0])+(Vpk[1][5][2]*
          Vpk[2][5][2])))+((mk[2]*((Vpk[1][4][0]*Vpk[2][4][0])+(Vpk[1][4][2]*
          Vpk[2][4][2])))+((mk[0]*(rk[0][2]*s2))+(mk[1]*((Vpk[1][3][0]*
          Vpk[2][3][0])+(Vpk[1][3][2]*Vpk[2][3][2])))))));
        mm[1][3] = ((mk[2]*((Vpk[1][4][0]*Vpk[3][4][0])+(Vpk[1][4][2]*
          Vpk[3][4][2])))-(mk[1]*(rk[1][2]*Vpk[1][3][0])));
        mm[1][4] = -(mk[2]*(rk[2][2]*Vpk[1][4][0]));
        mm[1][5] = ((mk[4]*((Vpk[1][6][0]*Vpk[5][6][0])+(Vpk[1][6][2]*
          Vpk[5][6][2])))-(mk[3]*(rk[3][2]*Vpk[1][5][0])));
        mm[1][6] = -(mk[4]*(rk[4][2]*Vpk[1][6][0]));
        mm[2][2] = ((ik[4][1][1]+(mk[4]*((Vpk[2][6][0]*Vpk[2][6][0])+(
          Vpk[2][6][2]*Vpk[2][6][2]))))+((ik[3][1][1]+(mk[3]*((Vpk[2][5][0]*
          Vpk[2][5][0])+(Vpk[2][5][2]*Vpk[2][5][2]))))+((ik[2][1][1]+(mk[2]*((
          Vpk[2][4][0]*Vpk[2][4][0])+(Vpk[2][4][2]*Vpk[2][4][2]))))+((
          ik[0][1][1]+(mk[0]*(rk[0][2]*rk[0][2])))+(ik[1][1][1]+(mk[1]*((
          Vpk[2][3][0]*Vpk[2][3][0])+(Vpk[2][3][2]*Vpk[2][3][2]))))))));
        mm[2][3] = ((ik[1][1][1]-(mk[1]*(rk[1][2]*Vpk[2][3][0])))+(ik[2][1][1]+(
          mk[2]*((Vpk[2][4][0]*Vpk[3][4][0])+(Vpk[2][4][2]*Vpk[3][4][2])))));
        mm[2][4] = (ik[2][1][1]-(mk[2]*(rk[2][2]*Vpk[2][4][0])));
        mm[2][5] = ((ik[3][1][1]-(mk[3]*(rk[3][2]*Vpk[2][5][0])))+(ik[4][1][1]+(
          mk[4]*((Vpk[2][6][0]*Vpk[5][6][0])+(Vpk[2][6][2]*Vpk[5][6][2])))));
        mm[2][6] = (ik[4][1][1]-(mk[4]*(rk[4][2]*Vpk[2][6][0])));
        mm[3][3] = ((ik[1][1][1]+(mk[1]*(rk[1][2]*rk[1][2])))+(ik[2][1][1]+(
          mk[2]*((Vpk[3][4][0]*Vpk[3][4][0])+(Vpk[3][4][2]*Vpk[3][4][2])))));
        mm[3][4] = (ik[2][1][1]-(mk[2]*(rk[2][2]*Vpk[3][4][0])));
        mm[3][5] = 0.;
        mm[3][6] = 0.;
        mm[4][4] = (ik[2][1][1]+(mk[2]*(rk[2][2]*rk[2][2])));
        mm[4][5] = 0.;
        mm[4][6] = 0.;
        mm[5][5] = ((ik[3][1][1]+(mk[3]*(rk[3][2]*rk[3][2])))+(ik[4][1][1]+(
          mk[4]*((Vpk[5][6][0]*Vpk[5][6][0])+(Vpk[5][6][2]*Vpk[5][6][2])))));
        mm[5][6] = (ik[4][1][1]-(mk[4]*(rk[4][2]*Vpk[5][6][0])));
        mm[6][6] = (ik[4][1][1]+(mk[4]*(rk[4][2]*rk[4][2])));
/*
Check for singular mass matrix
*/
        for (i = 0; i < 7; i++) {
            if (mm[i][i] < 1e-13) {
                b1cseterr(routine,47);
            }
        }
        mmflg = 1;
    }
/*
 Used 0.04 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   86 adds/subtracts/negates
                    138 multiplies
                      0 divides
                     28 assignments
*/
}

b1cdommldu(routine)
    int routine;
{
    int i;

    if (mmlduflg == 0) {
        b1cdomm(routine);
/*
Numerically decompose the mass matrix
*/
        b1cldudcomp(7,7,mmap,1e-13,workss,works,mm,mlo,mdi);
/*
Check for singular mass matrix
*/
        for (i = 0; i < 7; i++) {
            if (mdi[i] <= 1e-13) {
                b1cseterr(routine,47);
            }
        }
        mmlduflg = 1;
    }
}

b1clhs(routine)
    int routine;
{
/* Compute all remaining state- and force-dependent quantities
*/

    roustate = 2;
    b1cdommldu(routine);
    b1cdofs0();
}

b1cmassmat(mmat)
    double mmat[7][7];
{
/* Return the system mass matrix (LHS)
*/
    int i,j;

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(57,23);
        return;
    }
    b1cdomm(57);
    for (i = 0; i < 7; i++) {
        for (j = i; j <= 6; j++) {
            mmat[i][j] = mm[i][j];
            mmat[j][i] = mm[i][j];
        }
    }
}

b1cfrcmat(fmat)
    double fmat[7];
{
/* Return the system force matrix (RHS), excluding constraints
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(58,23);
        return;
    }
    b1cdofs0();
    for (i = 0; i < 7; i++) {
        fmat[i] = fs0[i];
    }
}

b1cmfrc(imult)
    double imult[4];
{
/*
Calculate forces due to constraint multipliers.

*/
    int i,j;
    double umult[4];

/*
Initialize all multiplier forces to zero.
*/
    for (i = 0; i <= 4; i++) {
        for (j = 0; j <= 2; j++) {
            mfk[i][j] = 0.;
            mtk[i][j] = 0.;
        }
    }
    for (i = 0; i <= 6; i++) {
        mtau[i] = 0.;
    }
/*
Compute user-generated multiplier forces
*/
    umult[0] = imult[0];
    umult[1] = imult[1];
    umult[2] = imult[2];
    umult[3] = imult[3];
    mfrcflg = 1;
    b1cuconsfrc(curtim,q,u,umult);
    mfrcflg = 0;
/*
 Used 0.00 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                     41 assignments
*/
}

b1cequivht(tau)
    double tau[7];
{
/* Compute tree hinge torques to match effect of applied loads
*/
    double fstareq[7][3],tstareq[7][3];

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(56,23);
        return;
    }
/*
Compute fstareq (forces)
*/
    fstareq[2][0] = -(ufk[0][0]+(9.81*(mk[0]*s2)));
    fstareq[2][2] = ((9.81*(mk[0]*c2))-ufk[0][2]);
    fstareq[3][0] = -(ufk[1][0]+(gk[3][0]*mk[1]));
    fstareq[3][2] = -(ufk[1][2]+(gk[3][2]*mk[1]));
    fstareq[4][0] = -(ufk[2][0]+(gk[4][0]*mk[2]));
    fstareq[4][2] = -(ufk[2][2]+(gk[4][2]*mk[2]));
    fstareq[5][0] = -(ufk[3][0]+(gk[5][0]*mk[3]));
    fstareq[5][2] = -(ufk[3][2]+(gk[5][2]*mk[3]));
    fstareq[6][0] = -(ufk[4][0]+(gk[6][0]*mk[4]));
    fstareq[6][2] = -(ufk[4][2]+(gk[6][2]*mk[4]));
/*
Compute tstareq (torques)
*/
/*
Compute taus (RHS ignoring constraints and inertial forces)
*/
    b1cdovpk();
    tau[0] = (utau[0]-(((fstareq[6][0]*Vpk[0][6][0])+(fstareq[6][2]*Vpk[0][6][2]
      ))+(((fstareq[5][0]*Vpk[0][5][0])+(fstareq[5][2]*Vpk[0][5][2]))+(((
      fstareq[4][0]*Vpk[0][4][0])+(fstareq[4][2]*Vpk[0][4][2]))+(((fstareq[2][0]
      *c2)+(fstareq[2][2]*s2))+((fstareq[3][0]*Vpk[0][3][0])+(fstareq[3][2]*
      Vpk[0][3][2])))))));
    tau[1] = (utau[1]-(((fstareq[6][0]*Vpk[1][6][0])+(fstareq[6][2]*Vpk[1][6][2]
      ))+(((fstareq[5][0]*Vpk[1][5][0])+(fstareq[5][2]*Vpk[1][5][2]))+(((
      fstareq[4][0]*Vpk[1][4][0])+(fstareq[4][2]*Vpk[1][4][2]))+(((fstareq[2][2]
      *c2)-(fstareq[2][0]*s2))+((fstareq[3][0]*Vpk[1][3][0])+(fstareq[3][2]*
      Vpk[1][3][2])))))));
    tau[2] = (utau[2]-((((fstareq[6][0]*Vpk[2][6][0])+(fstareq[6][2]*
      Vpk[2][6][2]))-utk[4][1])+((((fstareq[5][0]*Vpk[2][5][0])+(fstareq[5][2]*
      Vpk[2][5][2]))-utk[3][1])+((((fstareq[4][0]*Vpk[2][4][0])+(fstareq[4][2]*
      Vpk[2][4][2]))-utk[2][1])+((((fstareq[3][0]*Vpk[2][3][0])+(fstareq[3][2]*
      Vpk[2][3][2]))-utk[1][1])-(utk[0][1]+(fstareq[2][0]*rk[0][2])))))));
    tau[3] = (utau[3]-((((fstareq[4][0]*Vpk[3][4][0])+(fstareq[4][2]*
      Vpk[3][4][2]))-utk[2][1])-(utk[1][1]+(fstareq[3][0]*rk[1][2]))));
    tau[4] = (utau[4]+(utk[2][1]+(fstareq[4][0]*rk[2][2])));
    tau[5] = (utau[5]-((((fstareq[6][0]*Vpk[5][6][0])+(fstareq[6][2]*
      Vpk[5][6][2]))-utk[4][1])-(utk[3][1]+(fstareq[5][0]*rk[3][2]))));
    tau[6] = (utau[6]+(utk[4][1]+(fstareq[6][0]*rk[4][2])));
/*
Op counts below do not include called subroutines
*/
/*
 Used 0.02 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain   67 adds/subtracts/negates
                     49 multiplies
                      0 divides
                     17 assignments
*/
}

b1cfulltrq(udotin,multin,trqout)
    double udotin[7],multin[4],trqout[7];
{
/* Compute hinge torques which would produce indicated udots
*/
    double fstarr[7][3],tstarr[7][3],Otkr[7][3],Atir[7][3],Atkr[7][3];

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(61,23);
        return;
    }
/*
Compute multiplier-generated forces
*/
    b1cmfrc(multin);
/*
Account for inertial accelerations and supplied udots
*/
    Otkr[3][1] = (udotin[2]+udotin[3]);
    Otkr[4][1] = (Otkr[3][1]+udotin[4]);
    Otkr[5][1] = (udotin[2]+udotin[5]);
    Otkr[6][1] = (Otkr[5][1]+udotin[6]);
    Atkr[2][0] = (((udotin[0]*c2)-(udotin[1]*s2))-(rk[0][2]*udotin[2]));
    Atkr[2][2] = (((udotin[0]*s2)+(udotin[1]*c2))-(u[2]*Wkrpk[2][0]));
    Atir[3][0] = (Atkr[2][0]+(ri[1][2]*udotin[2]));
    Atir[3][2] = (Atkr[2][2]-(u[2]*Wirk[3][0]));
    Atkr[3][0] = (((Atir[3][0]*c3)-(Atir[3][2]*s3))-(Otkr[3][1]*rk[1][2]));
    Atkr[3][2] = (((Atir[3][0]*s3)+(Atir[3][2]*c3))-(wk[3][1]*Wkrpk[3][0]));
    Atir[4][0] = (Atkr[3][0]+(Otkr[3][1]*ri[2][2]));
    Atir[4][2] = (Atkr[3][2]-(Wirk[4][0]*wk[3][1]));
    Atkr[4][0] = (((Atir[4][0]*c4)-(Atir[4][2]*s4))-(Otkr[4][1]*rk[2][2]));
    Atkr[4][2] = (((Atir[4][0]*s4)+(Atir[4][2]*c4))-(wk[4][1]*Wkrpk[4][0]));
    Atir[5][0] = (Atkr[2][0]+(ri[3][2]*udotin[2]));
    Atir[5][2] = (Atkr[2][2]-(u[2]*Wirk[5][0]));
    Atkr[5][0] = (((Atir[5][0]*c5)-(Atir[5][2]*s5))-(Otkr[5][1]*rk[3][2]));
    Atkr[5][2] = (((Atir[5][0]*s5)+(Atir[5][2]*c5))-(wk[5][1]*Wkrpk[5][0]));
    Atir[6][0] = (Atkr[5][0]+(Otkr[5][1]*ri[4][2]));
    Atir[6][2] = (Atkr[5][2]-(Wirk[6][0]*wk[5][1]));
    Atkr[6][0] = (((Atir[6][0]*c6)-(Atir[6][2]*s6))-(Otkr[6][1]*rk[4][2]));
    Atkr[6][2] = (((Atir[6][0]*s6)+(Atir[6][2]*c6))-(wk[6][1]*Wkrpk[6][0]));
/*
Accumulate all forces and torques
*/
    fstarr[2][0] = ((mfk[0][0]+ufk[0][0])+(mk[0]*((9.81*s2)-Atkr[2][0])));
    fstarr[2][1] = (mfk[0][1]+ufk[0][1]);
    fstarr[2][2] = ((mfk[0][2]+ufk[0][2])-(mk[0]*(Atkr[2][2]+(9.81*c2))));
    fstarr[3][0] = ((mfk[1][0]+ufk[1][0])+(mk[1]*(gk[3][0]-Atkr[3][0])));
    fstarr[3][1] = (mfk[1][1]+ufk[1][1]);
    fstarr[3][2] = ((mfk[1][2]+ufk[1][2])+(mk[1]*(gk[3][2]-Atkr[3][2])));
    fstarr[4][0] = ((mfk[2][0]+ufk[2][0])+(mk[2]*(gk[4][0]-Atkr[4][0])));
    fstarr[4][1] = (mfk[2][1]+ufk[2][1]);
    fstarr[4][2] = ((mfk[2][2]+ufk[2][2])+(mk[2]*(gk[4][2]-Atkr[4][2])));
    fstarr[5][0] = ((mfk[3][0]+ufk[3][0])+(mk[3]*(gk[5][0]-Atkr[5][0])));
    fstarr[5][1] = (mfk[3][1]+ufk[3][1]);
    fstarr[5][2] = ((mfk[3][2]+ufk[3][2])+(mk[3]*(gk[5][2]-Atkr[5][2])));
    fstarr[6][0] = ((mfk[4][0]+ufk[4][0])+(mk[4]*(gk[6][0]-Atkr[6][0])));
    fstarr[6][1] = (mfk[4][1]+ufk[4][1]);
    fstarr[6][2] = ((mfk[4][2]+ufk[4][2])+(mk[4]*(gk[6][2]-Atkr[6][2])));
    tstarr[2][0] = (mtk[0][0]+utk[0][0]);
    tstarr[2][1] = ((mtk[0][1]+utk[0][1])-(ik[0][1][1]*udotin[2]));
    tstarr[2][2] = (mtk[0][2]+utk[0][2]);
    tstarr[3][0] = (mtk[1][0]+utk[1][0]);
    tstarr[3][1] = ((mtk[1][1]+utk[1][1])-(ik[1][1][1]*Otkr[3][1]));
    tstarr[3][2] = (mtk[1][2]+utk[1][2]);
    tstarr[4][0] = (mtk[2][0]+utk[2][0]);
    tstarr[4][1] = ((mtk[2][1]+utk[2][1])-(ik[2][1][1]*Otkr[4][1]));
    tstarr[4][2] = (mtk[2][2]+utk[2][2]);
    tstarr[5][0] = (mtk[3][0]+utk[3][0]);
    tstarr[5][1] = ((mtk[3][1]+utk[3][1])-(ik[3][1][1]*Otkr[5][1]));
    tstarr[5][2] = (mtk[3][2]+utk[3][2]);
    tstarr[6][0] = (mtk[4][0]+utk[4][0]);
    tstarr[6][1] = ((mtk[4][1]+utk[4][1])-(ik[4][1][1]*Otkr[6][1]));
    tstarr[6][2] = (mtk[4][2]+utk[4][2]);
/*
Now calculate the torques
*/
    b1cdovpk();
    trqout[0] = -((mtau[0]+utau[0])+(((fstarr[6][0]*Vpk[0][6][0])+(fstarr[6][2]*
      Vpk[0][6][2]))+(((fstarr[5][0]*Vpk[0][5][0])+(fstarr[5][2]*Vpk[0][5][2]))+
      (((fstarr[4][0]*Vpk[0][4][0])+(fstarr[4][2]*Vpk[0][4][2]))+(((fstarr[2][0]
      *c2)+(fstarr[2][2]*s2))+((fstarr[3][0]*Vpk[0][3][0])+(fstarr[3][2]*
      Vpk[0][3][2])))))));
    trqout[1] = -((mtau[1]+utau[1])+(((fstarr[6][0]*Vpk[1][6][0])+(fstarr[6][2]*
      Vpk[1][6][2]))+(((fstarr[5][0]*Vpk[1][5][0])+(fstarr[5][2]*Vpk[1][5][2]))+
      (((fstarr[4][0]*Vpk[1][4][0])+(fstarr[4][2]*Vpk[1][4][2]))+(((fstarr[2][2]
      *c2)-(fstarr[2][0]*s2))+((fstarr[3][0]*Vpk[1][3][0])+(fstarr[3][2]*
      Vpk[1][3][2])))))));
    trqout[2] = -((mtau[2]+utau[2])+((tstarr[6][1]+((fstarr[6][0]*Vpk[2][6][0])+
      (fstarr[6][2]*Vpk[2][6][2])))+((tstarr[5][1]+((fstarr[5][0]*Vpk[2][5][0])+
      (fstarr[5][2]*Vpk[2][5][2])))+((tstarr[4][1]+((fstarr[4][0]*Vpk[2][4][0])+
      (fstarr[4][2]*Vpk[2][4][2])))+((tstarr[2][1]-(fstarr[2][0]*rk[0][2]))+(
      tstarr[3][1]+((fstarr[3][0]*Vpk[2][3][0])+(fstarr[3][2]*Vpk[2][3][2]))))))
      ));
    trqout[3] = -((mtau[3]+utau[3])+((tstarr[3][1]-(fstarr[3][0]*rk[1][2]))+(
      tstarr[4][1]+((fstarr[4][0]*Vpk[3][4][0])+(fstarr[4][2]*Vpk[3][4][2])))));
    trqout[4] = -((mtau[4]+utau[4])+(tstarr[4][1]-(fstarr[4][0]*rk[2][2])));
    trqout[5] = -((mtau[5]+utau[5])+((tstarr[5][1]-(fstarr[5][0]*rk[3][2]))+(
      tstarr[6][1]+((fstarr[6][0]*Vpk[5][6][0])+(fstarr[6][2]*Vpk[5][6][2])))));
    trqout[6] = -((mtau[6]+utau[6])+(tstarr[6][1]-(fstarr[6][0]*rk[4][2])));
/*
Op counts below do not include called subroutines
*/
/*
 Used 0.06 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain  149 adds/subtracts/negates
                     92 multiplies
                      0 divides
                     59 assignments
*/
}

b1ccomptrq(udotin,trqout)
    double udotin[7],trqout[7];
{
/* Compute hinge torques to produce these udots, ignoring constraints
*/
    int i;
    double multin[4];

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(60,23);
        return;
    }
    for (i = 0; i < 4; i++) {
        multin[i] = 0.;
    }
    b1cfulltrq(udotin,multin,trqout);
}

b1cmulttrq(multin,trqout)
    double multin[4],trqout[7];
{
/* Compute hinge trqs which would be produced by these mults.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(65,23);
        return;
    }
    b1cmfrc(multin);
    b1cfsmult();
    for (i = 0; i < 7; i++) {
        trqout[i] = fs[i];
    }
}

b1cfs0()
{

/*
Compute Fs (ignoring multiplier forces)
*/
    fs[0] = fs0[0];
    fs[1] = fs0[1];
    fs[2] = fs0[2];
    fs[3] = fs0[3];
    fs[4] = fs0[4];
    fs[5] = fs0[5];
    fs[6] = fs0[6];
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      7 assignments
*/
}

b1cfsmult()
{

/*
Compute Fs (multiplier-generated forces only)
*/
    b1cdovpk();
    fs[0] = (mtau[0]+(((mfk[4][0]*Vpk[0][6][0])+(mfk[4][2]*Vpk[0][6][2]))+(((
      mfk[3][0]*Vpk[0][5][0])+(mfk[3][2]*Vpk[0][5][2]))+(((mfk[2][0]*
      Vpk[0][4][0])+(mfk[2][2]*Vpk[0][4][2]))+(((mfk[0][0]*c2)+(mfk[0][2]*s2))+(
      (mfk[1][0]*Vpk[0][3][0])+(mfk[1][2]*Vpk[0][3][2])))))));
    fs[1] = (mtau[1]-((((((mfk[0][0]*s2)-(mfk[0][2]*c2))-((mfk[1][0]*
      Vpk[1][3][0])+(mfk[1][2]*Vpk[1][3][2])))-((mfk[2][0]*Vpk[1][4][0])+(
      mfk[2][2]*Vpk[1][4][2])))-((mfk[3][0]*Vpk[1][5][0])+(mfk[3][2]*
      Vpk[1][5][2])))-((mfk[4][0]*Vpk[1][6][0])+(mfk[4][2]*Vpk[1][6][2]))));
    fs[2] = (mtau[2]-((((((mfk[0][0]*rk[0][2])-mtk[0][1])-(mtk[1][1]+((mfk[1][0]
      *Vpk[2][3][0])+(mfk[1][2]*Vpk[2][3][2]))))-(mtk[2][1]+((mfk[2][0]*
      Vpk[2][4][0])+(mfk[2][2]*Vpk[2][4][2]))))-(mtk[3][1]+((mfk[3][0]*
      Vpk[2][5][0])+(mfk[3][2]*Vpk[2][5][2]))))-(mtk[4][1]+((mfk[4][0]*
      Vpk[2][6][0])+(mfk[4][2]*Vpk[2][6][2])))));
    fs[3] = (mtau[3]-(((mfk[1][0]*rk[1][2])-mtk[1][1])-(mtk[2][1]+((mfk[2][0]*
      Vpk[3][4][0])+(mfk[2][2]*Vpk[3][4][2])))));
    fs[4] = (mtau[4]-((mfk[2][0]*rk[2][2])-mtk[2][1]));
    fs[5] = (mtau[5]-(((mfk[3][0]*rk[3][2])-mtk[3][1])-(mtk[4][1]+((mfk[4][0]*
      Vpk[5][6][0])+(mfk[4][2]*Vpk[5][6][2])))));
    fs[6] = (mtau[6]-((mfk[4][0]*rk[4][2])-mtk[4][1]));
/*
 Used 0.02 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   48 adds/subtracts/negates
                     37 multiplies
                      0 divides
                      7 assignments
*/
}

b1cfsfull()
{

/*
Compute Fs (including all forces)
*/
    b1cfsmult();
    fs[0] = (fs[0]+fs0[0]);
    fs[1] = (fs[1]+fs0[1]);
    fs[2] = (fs[2]+fs0[2]);
    fs[3] = (fs[3]+fs0[3]);
    fs[4] = (fs[4]+fs0[4]);
    fs[5] = (fs[5]+fs0[5]);
    fs[6] = (fs[6]+fs0[6]);
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    7 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      7 assignments
*/
}

b1cfsgenmult()
{

/*
Compute Fs (generic multiplier-generated forces)
*/
    b1cdovpk();
    fs[0] = (mtau[0]+(((mfk[4][0]*Vpk[0][6][0])+(mfk[4][2]*Vpk[0][6][2]))+(((
      mfk[3][0]*Vpk[0][5][0])+(mfk[3][2]*Vpk[0][5][2]))+(((mfk[2][0]*
      Vpk[0][4][0])+(mfk[2][2]*Vpk[0][4][2]))+(((mfk[0][0]*c2)+(mfk[0][2]*s2))+(
      (mfk[1][0]*Vpk[0][3][0])+(mfk[1][2]*Vpk[0][3][2])))))));
    fs[1] = (mtau[1]-((((((mfk[0][0]*s2)-(mfk[0][2]*c2))-((mfk[1][0]*
      Vpk[1][3][0])+(mfk[1][2]*Vpk[1][3][2])))-((mfk[2][0]*Vpk[1][4][0])+(
      mfk[2][2]*Vpk[1][4][2])))-((mfk[3][0]*Vpk[1][5][0])+(mfk[3][2]*
      Vpk[1][5][2])))-((mfk[4][0]*Vpk[1][6][0])+(mfk[4][2]*Vpk[1][6][2]))));
    fs[2] = (mtau[2]-((((((mfk[0][0]*rk[0][2])-mtk[0][1])-(mtk[1][1]+((mfk[1][0]
      *Vpk[2][3][0])+(mfk[1][2]*Vpk[2][3][2]))))-(mtk[2][1]+((mfk[2][0]*
      Vpk[2][4][0])+(mfk[2][2]*Vpk[2][4][2]))))-(mtk[3][1]+((mfk[3][0]*
      Vpk[2][5][0])+(mfk[3][2]*Vpk[2][5][2]))))-(mtk[4][1]+((mfk[4][0]*
      Vpk[2][6][0])+(mfk[4][2]*Vpk[2][6][2])))));
    fs[3] = (mtau[3]-(((mfk[1][0]*rk[1][2])-mtk[1][1])-(mtk[2][1]+((mfk[2][0]*
      Vpk[3][4][0])+(mfk[2][2]*Vpk[3][4][2])))));
    fs[4] = (mtau[4]-((mfk[2][0]*rk[2][2])-mtk[2][1]));
    fs[5] = (mtau[5]-(((mfk[3][0]*rk[3][2])-mtk[3][1])-(mtk[4][1]+((mfk[4][0]*
      Vpk[5][6][0])+(mfk[4][2]*Vpk[5][6][2])))));
    fs[6] = (mtau[6]-((mfk[4][0]*rk[4][2])-mtk[4][1]));
/*
 Used 0.02 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   48 adds/subtracts/negates
                     37 multiplies
                      0 divides
                      7 assignments
*/
}

b1cfsgenfull()
{

/*
Compute Fs (incl generic mult & other forces)
*/
    b1cfsgenmult();
    fs[0] = (fs[0]+fs0[0]);
    fs[1] = (fs[1]+fs0[1]);
    fs[2] = (fs[2]+fs0[2]);
    fs[3] = (fs[3]+fs0[3]);
    fs[4] = (fs[4]+fs0[4]);
    fs[5] = (fs[5]+fs0[5]);
    fs[6] = (fs[6]+fs0[6]);
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    7 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      7 assignments
*/
}

b1crhs()
{
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

/*
Compute hinge torques for tree hinges
*/
    tauc[0] = (mtau[0]+utau[0]);
    tauc[1] = (mtau[1]+utau[1]);
    tauc[2] = (mtau[2]+utau[2]);
    tauc[3] = (mtau[3]+utau[3]);
    tauc[4] = (mtau[4]+utau[4]);
    tauc[5] = (mtau[5]+utau[5]);
    tauc[6] = (mtau[6]+utau[6]);
    b1cdoiner();
/*
Compute onk & onb (angular accels in N)
*/
    Onkb[3][1] = (udot[2]+udot[3]);
    Onkb[4][1] = (Onkb[3][1]+udot[4]);
    Onkb[5][1] = (udot[2]+udot[5]);
    Onkb[6][1] = (Onkb[5][1]+udot[6]);
    onb[0][0] = 0.;
    onb[0][1] = udot[2];
    onb[0][2] = 0.;
    onb[1][0] = 0.;
    onb[1][1] = Onkb[3][1];
    onb[1][2] = 0.;
    onb[2][0] = 0.;
    onb[2][1] = Onkb[4][1];
    onb[2][2] = 0.;
    onb[3][0] = 0.;
    onb[3][1] = Onkb[5][1];
    onb[3][2] = 0.;
    onb[4][0] = 0.;
    onb[4][1] = Onkb[6][1];
    onb[4][2] = 0.;
/*
Compute acceleration dyadics
*/
    dyad[0][0][0] = -w1w1[0];
    dyad[0][0][1] = 0.;
    dyad[0][0][2] = udot[2];
    dyad[0][1][0] = 0.;
    dyad[0][1][1] = 0.;
    dyad[0][1][2] = 0.;
    dyad[0][2][0] = -udot[2];
    dyad[0][2][1] = 0.;
    dyad[0][2][2] = -w1w1[0];
    dyad[1][0][0] = -w1w1[1];
    dyad[1][0][1] = 0.;
    dyad[1][0][2] = Onkb[3][1];
    dyad[1][1][0] = 0.;
    dyad[1][1][1] = 0.;
    dyad[1][1][2] = 0.;
    dyad[1][2][0] = -Onkb[3][1];
    dyad[1][2][1] = 0.;
    dyad[1][2][2] = -w1w1[1];
    dyad[2][0][0] = -w1w1[2];
    dyad[2][0][1] = 0.;
    dyad[2][0][2] = Onkb[4][1];
    dyad[2][1][0] = 0.;
    dyad[2][1][1] = 0.;
    dyad[2][1][2] = 0.;
    dyad[2][2][0] = -Onkb[4][1];
    dyad[2][2][1] = 0.;
    dyad[2][2][2] = -w1w1[2];
    dyad[3][0][0] = -w1w1[3];
    dyad[3][0][1] = 0.;
    dyad[3][0][2] = Onkb[5][1];
    dyad[3][1][0] = 0.;
    dyad[3][1][1] = 0.;
    dyad[3][1][2] = 0.;
    dyad[3][2][0] = -Onkb[5][1];
    dyad[3][2][1] = 0.;
    dyad[3][2][2] = -w1w1[3];
    dyad[4][0][0] = -w1w1[4];
    dyad[4][0][1] = 0.;
    dyad[4][0][2] = Onkb[6][1];
    dyad[4][1][0] = 0.;
    dyad[4][1][1] = 0.;
    dyad[4][1][2] = 0.;
    dyad[4][2][0] = -Onkb[6][1];
    dyad[4][2][1] = 0.;
    dyad[4][2][2] = -w1w1[4];
/*
Compute ank & anb (mass center linear accels in N)
*/
    Ankb[2][0] = (((udot[0]*c2)-(udot[1]*s2))-(rk[0][2]*udot[2]));
    Ankb[2][2] = ((udot[0]*s2)+(udot[1]*c2));
    AOnkri[3][0] = (Ankb[2][0]+(ri[1][2]*udot[2]));
    Ankb[3][0] = (((AOnkri[3][0]*c3)-(Ankb[2][2]*s3))-(Onkb[3][1]*rk[1][2]));
    Ankb[3][2] = ((Ankb[2][2]*c3)+(AOnkri[3][0]*s3));
    AOnkri[4][0] = (Ankb[3][0]+(Onkb[3][1]*ri[2][2]));
    Ankb[4][0] = (((AOnkri[4][0]*c4)-(Ankb[3][2]*s4))-(Onkb[4][1]*rk[2][2]));
    Ankb[4][2] = ((Ankb[3][2]*c4)+(AOnkri[4][0]*s4));
    AOnkri[5][0] = (Ankb[2][0]+(ri[3][2]*udot[2]));
    Ankb[5][0] = (((AOnkri[5][0]*c5)-(Ankb[2][2]*s5))-(Onkb[5][1]*rk[3][2]));
    Ankb[5][2] = ((Ankb[2][2]*c5)+(AOnkri[5][0]*s5));
    AOnkri[6][0] = (Ankb[5][0]+(Onkb[5][1]*ri[4][2]));
    Ankb[6][0] = (((AOnkri[6][0]*c6)-(Ankb[5][2]*s6))-(Onkb[6][1]*rk[4][2]));
    Ankb[6][2] = ((Ankb[5][2]*c6)+(AOnkri[6][0]*s6));
    AnkAtk[2][2] = (Ankb[2][2]+Atk[2][2]);
    ank[2][0] = ((AnkAtk[2][2]*s2)+(Ankb[2][0]*c2));
    ank[2][2] = ((AnkAtk[2][2]*c2)-(Ankb[2][0]*s2));
    AnkAtk[3][0] = (Ankb[3][0]+Atk[3][0]);
    AnkAtk[3][2] = (Ankb[3][2]+Atk[3][2]);
    ank[3][0] = ((AnkAtk[3][0]*cnk[3][0][0])+(AnkAtk[3][2]*cnk[3][0][2]));
    ank[3][2] = ((AnkAtk[3][0]*cnk[3][2][0])+(AnkAtk[3][2]*cnk[3][2][2]));
    AnkAtk[4][0] = (Ankb[4][0]+Atk[4][0]);
    AnkAtk[4][2] = (Ankb[4][2]+Atk[4][2]);
    ank[4][0] = ((AnkAtk[4][0]*cnk[4][0][0])+(AnkAtk[4][2]*cnk[4][0][2]));
    ank[4][2] = ((AnkAtk[4][0]*cnk[4][2][0])+(AnkAtk[4][2]*cnk[4][2][2]));
    AnkAtk[5][0] = (Ankb[5][0]+Atk[5][0]);
    AnkAtk[5][2] = (Ankb[5][2]+Atk[5][2]);
    ank[5][0] = ((AnkAtk[5][0]*cnk[5][0][0])+(AnkAtk[5][2]*cnk[5][0][2]));
    ank[5][2] = ((AnkAtk[5][0]*cnk[5][2][0])+(AnkAtk[5][2]*cnk[5][2][2]));
    AnkAtk[6][0] = (Ankb[6][0]+Atk[6][0]);
    AnkAtk[6][2] = (Ankb[6][2]+Atk[6][2]);
    ank[6][0] = ((AnkAtk[6][0]*cnk[6][0][0])+(AnkAtk[6][2]*cnk[6][0][2]));
    ank[6][2] = ((AnkAtk[6][0]*cnk[6][2][0])+(AnkAtk[6][2]*cnk[6][2][2]));
    anb[0][0] = ank[2][0];
    anb[0][1] = 0.;
    anb[0][2] = ank[2][2];
    anb[1][0] = ank[3][0];
    anb[1][1] = 0.;
    anb[1][2] = ank[3][2];
    anb[2][0] = ank[4][0];
    anb[2][1] = 0.;
    anb[2][2] = ank[4][2];
    anb[3][0] = ank[5][0];
    anb[3][1] = 0.;
    anb[3][2] = ank[5][2];
    anb[4][0] = ank[6][0];
    anb[4][1] = 0.;
    anb[4][2] = ank[6][2];
/*
Compute constraint acceleration errors
*/
    roustate = 3;
    b1cuaerr(curtim,q,u,udot,&aerr[0]);
/*
 Used 0.04 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   64 adds/subtracts/negates
                     49 multiplies
                      0 divides
                    119 assignments
*/
}

b1cpseudo(lqout,luout)
    double lqout[1],luout[1];
{
/*
Return pseudo-coordinates for loop joints.

*/
/*
There are no loop joints in this system.

*/
}

b1cpsqdot(lqdout)
    double lqdout[1];
{
/*
Return pseudo-coordinate derivatives for loop joints.

*/
/*
There are no loop joints in this system.

*/
}

b1cpsudot(ludout)
    double ludout[1];
{
/*
Return pseudo-coordinate accelerations for loop joints.

*/
/*
There are no loop joints in this system.

*/
}

b1cperr(errs)
    double errs[4];
{
/*
Return position constraint errors.

*/

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(26,23);
        return;
    }
    errs[0] = perr[0];
    errs[1] = perr[1];
    errs[2] = perr[2];
    errs[3] = perr[3];
}

b1cverr(errs)
    double errs[4];
{
/*
Return velocity constraint errors.

*/

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(27,23);
        return;
    }
    errs[0] = verr[0];
    errs[1] = verr[1];
    errs[2] = verr[2];
    errs[3] = verr[3];
}

b1caerr(errs)
    double errs[4];
{
/*
Return acceleration constraint errors.

*/

    if (roustate != 3) {
        b1cseterr(35,24);
        return;
    }
    errs[0] = aerr[0];
    errs[1] = aerr[1];
    errs[2] = aerr[2];
    errs[3] = aerr[3];
}
int 
b1cindx(joint,axis)
    int joint,axis;
{
    int offs,gotit;

    if (b1cchkjaxis(36,joint,axis) != 0) {
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

b1cpresacc(joint,axis,prval)
    int joint,axis;
    double prval;
{

}

b1cpresvel(joint,axis,prval)
    int joint,axis;
    double prval;
{

}

b1cprespos(joint,axis,prval)
    int joint,axis;
    double prval;
{

}

b1cgetht(joint,axis,torque)
    int joint,axis;
    double *torque;
{

    if (b1cchkjaxis(30,joint,axis) != 0) {
        return;
    }
    if (roustate != 3) {
        b1cseterr(30,24);
        return;
    }
    *torque = tauc[b1cindx(joint,axis)];
}

b1chinget(joint,axis,torque)
    int joint,axis;
    double torque;
{

    if (b1cchkjaxis(10,joint,axis) != 0) {
        return;
    }
    if (roustate != 2) {
        b1cseterr(10,23);
        return;
    }
    if (mfrcflg != 0) {
        mtau[b1cindx(joint,axis)] = mtau[b1cindx(joint,axis)]+torque;
    } else {
        fs0flg = 0;
        utau[b1cindx(joint,axis)] = utau[b1cindx(joint,axis)]+torque;
    }
}

b1cpointf(body,point,force)
    int body;
    double point[3],force[3];
{
    double torque[3];

    if (b1cchkbnum(11,body) != 0) {
        return;
    }
    if (roustate != 2) {
        b1cseterr(11,23);
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

b1cbodyt(body,torque)
    int body;
    double torque[3];
{

    if (b1cchkbnum(12,body) != 0) {
        return;
    }
    if (roustate != 2) {
        b1cseterr(12,23);
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

b1cdoww(routine)
    int routine;
{
    double pp[4][7],dpp[7][4];
    int i,j,c;
    double sum;
    double dfk[5][3],dtk[5][3],dtau[7],dltci[1][3],dltc[1][3],dlfci[1][3],dlfc[1
      ][3];
    double dTinb[1][3],dToutb[1][3],dltaufi[1][3],dltaufo[1][3],dltauti[1][3],
      dltauto[1][3];
    double umult[4];
    double dfs[7],row[7],dinvrow[7];

    roustate = 2;
    if (wwflg == 0) {
/*
Compute constraint effects
*/
        b1cdovpk();
        b1cdommldu(routine);
/*
Constraint 0 (user constraint)
*/
/*
Initialize all multiplier forces to zero.
*/
        for (i = 0; i <= 4; i++) {
            for (j = 0; j <= 2; j++) {
                mfk[i][j] = 0.;
                mtk[i][j] = 0.;
            }
        }
        for (i = 0; i <= 6; i++) {
            mtau[i] = 0.;
        }
/*
Compute user-generated multiplier forces
*/
        umult[0] = 1.;
        umult[1] = 0.;
        umult[2] = 0.;
        umult[3] = 0.;
        mfrcflg = 1;
        b1cuconsfrc(curtim,q,u,umult);
        mfrcflg = 0;
        b1cfsmult();
        b1cldubsl(7,7,mmap,mlo,fs,row);
        b1cldubsd(7,7,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 6; i++) {
            pp[0][i] = row[i];
            dpp[i][0] = dinvrow[i];
        }
        wmap[0] = 0;
/*
Constraint 1 (user constraint)
*/
/*
Initialize all multiplier forces to zero.
*/
        for (i = 0; i <= 4; i++) {
            for (j = 0; j <= 2; j++) {
                mfk[i][j] = 0.;
                mtk[i][j] = 0.;
            }
        }
        for (i = 0; i <= 6; i++) {
            mtau[i] = 0.;
        }
/*
Compute user-generated multiplier forces
*/
        umult[0] = 0.;
        umult[1] = 1.;
        umult[2] = 0.;
        umult[3] = 0.;
        mfrcflg = 1;
        b1cuconsfrc(curtim,q,u,umult);
        mfrcflg = 0;
        b1cfsmult();
        b1cldubsl(7,7,mmap,mlo,fs,row);
        b1cldubsd(7,7,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 6; i++) {
            pp[1][i] = row[i];
            dpp[i][1] = dinvrow[i];
        }
        wmap[1] = 1;
/*
Constraint 2 (user constraint)
*/
/*
Initialize all multiplier forces to zero.
*/
        for (i = 0; i <= 4; i++) {
            for (j = 0; j <= 2; j++) {
                mfk[i][j] = 0.;
                mtk[i][j] = 0.;
            }
        }
        for (i = 0; i <= 6; i++) {
            mtau[i] = 0.;
        }
/*
Compute user-generated multiplier forces
*/
        umult[0] = 0.;
        umult[1] = 0.;
        umult[2] = 1.;
        umult[3] = 0.;
        mfrcflg = 1;
        b1cuconsfrc(curtim,q,u,umult);
        mfrcflg = 0;
        b1cfsmult();
        b1cldubsl(7,7,mmap,mlo,fs,row);
        b1cldubsd(7,7,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 6; i++) {
            pp[2][i] = row[i];
            dpp[i][2] = dinvrow[i];
        }
        wmap[2] = 2;
/*
Constraint 3 (user constraint)
*/
/*
Initialize all multiplier forces to zero.
*/
        for (i = 0; i <= 4; i++) {
            for (j = 0; j <= 2; j++) {
                mfk[i][j] = 0.;
                mtk[i][j] = 0.;
            }
        }
        for (i = 0; i <= 6; i++) {
            mtau[i] = 0.;
        }
/*
Compute user-generated multiplier forces
*/
        umult[0] = 0.;
        umult[1] = 0.;
        umult[2] = 0.;
        umult[3] = 1.;
        mfrcflg = 1;
        b1cuconsfrc(curtim,q,u,umult);
        mfrcflg = 0;
        b1cfsmult();
        b1cldubsl(7,7,mmap,mlo,fs,row);
        b1cldubsd(7,7,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 6; i++) {
            pp[3][i] = row[i];
            dpp[i][3] = dinvrow[i];
        }
        wmap[3] = 3;
/*
Produce constraint coefficient matrix WW
*/
        for (c = 0; c <= 3; c++) {
            for (i = c; i <= 3; i++) {
                sum = 0.;
                for (j = 0; j <= 6; j++) {
                    sum = sum+pp[wmap[c]][j]*dpp[j][wmap[i]];
                }
                ww[wmap[c]][wmap[i]] = sum;
                ww[wmap[i]][wmap[c]] = sum;
            }
        }
/*
Form QR decomposition of WW
*/
        b1cqrdcomp(4,4,4,4,wmap,wmap,ww,qraux,jpvt);
        wwflg = 1;
    }
/*
 Used 0.01 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   70 adds/subtracts/negates
                     70 multiplies
                      0 divides
                    320 assignments
*/
}

b1cxudot0(routine,oudot0)
    int routine;
    double oudot0[7];
{
/*
Compute unconstrained equations
*/
    int i;

    b1clhs(routine);
/*
Solve equations ignoring constraints
*/
    b1cfs0();
    b1cldubslv(7,7,mmap,works,mlo,mdi,fs,udot);
    for (i = 0; i <= 6; i++) {
        oudot0[i] = udot[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      7 assignments
*/
}

b1cudot0(oudot0)
    double oudot0[7];
{

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(66,23);
        return;
    }
    b1cxudot0(66,oudot0);
}

b1csetudot(iudot)
    double iudot[7];
{
/*
Assign udots and advance to stage Dynamics Ready
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(68,23);
        return;
    }
    for (i = 0; i <= 6; i++) {
        udot[i] = iudot[i];
    }
    b1crhs();
}

b1cxudotm(routine,imult,oudotm)
    int routine;
    double imult[4],oudotm[7];
{
/*
Compute udots due only to multipliers
*/
    int i;

    b1clhs(routine);
    b1cmfrc(imult);
    b1cfsmult();
    b1cldubslv(7,7,mmap,works,mlo,mdi,fs,udot);
    for (i = 0; i <= 6; i++) {
        oudotm[i] = udot[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      7 assignments
*/
}

b1cudotm(imult,oudotm)
    double imult[4],oudotm[7];
{

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(67,23);
        return;
    }
    b1cxudotm(67,imult,oudotm);
}

b1cderiv(oqdot,oudot)
    double oqdot[7],oudot[7];
{
/*
This is the derivative section for a 5-body ground-based
system with 7 hinge degree(s) of freedom.
There are 4 constraints.
*/
    double workr[4],bb[4],b0[4],v0[4],p0[4];
    int iwork[4];
    int i,j;
    double udot0[7],udot1[7];

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(17,23);
        return;
    }
    if (stabvelq == 1) {
        b1cseterr(17,32);
    }
    if (stabposq == 1) {
        b1cseterr(17,33);
    }
    wsiz = 4;
/*
Compute unconstrained equations
*/
    b1cxudot0(17,udot0);
    b1crhs();
    b1caerr(b0);
    if (stabvel  !=  0.) {
        b1cverr(v0);
    }
    if (stabpos  !=  0.) {
        b1cperr(p0);
    }
/*
Stabilize constraints using Baumgarte's method
*/
    for (i = 0; i <= 3; i++) {
        bb[i] = -b0[i];
    }
    if (stabvel  !=  0.) {
        for (i = 0; i <= 3; i++) {
            bb[i] = bb[i]-stabvel*v0[i];
        }
    }
    if (stabpos  !=  0.) {
        for (i = 0; i <= 3; i++) {
            bb[i] = bb[i]-stabpos*p0[i];
        }
    }
/*
Compute and decompose constraint matrix WW
*/
    b1cdoww(17);
/*
Numerically solve for constraint multipliers
*/
    b1cqrbslv(4,4,4,4,wmap,wmap,1e-13,workr,iwork,ww,qraux,jpvt,bb,mult,&wrank);
    for (i = 0; i <= 3; i++) {
        multmap[i] = 0;
    }
    for (i = 0; i < wrank; i++) {
        multmap[jpvt[i]] = 1;
    }
    j = 0;
    for (i = 0; i <= 3; i++) {
        if (multmap[i] != 0) {
            multmap[j] = wmap[i];
            j = j+1;
        }
    }
/*
Compute final udots
*/
    b1cxudotm(17,mult,udot1);
    for (i = 0; i <= 6; i++) {
        udot[i] = udot0[i]+udot1[i];
    }
    b1crhs();
    for (i = 0; i <= 6; i++) {
        oqdot[i] = qdot[i];
    }
    for (i = 0; i <= 6; i++) {
        oudot[i] = udot[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   19 adds/subtracts/negates
                      8 multiplies
                      0 divides
                     33 assignments
*/
}
/*
Compute residuals for use with DAE integrator
*/

b1cresid(eqdot,eudot,emults,resid)
    double eqdot[7],eudot[7],emults[4],resid[18];
{
    int i;
    double uderrs[7],p0[4];

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(16,23);
        return;
    }
    if (stabposq == 1) {
        b1cseterr(16,33);
    }
    b1cfulltrq(eudot,emults,uderrs);
    for (i = 0; i < 7; i++) {
        resid[i] = eqdot[i]-qdot[i];
    }
    for (i = 0; i < 7; i++) {
        resid[7+i] = uderrs[i];
    }
    b1cverr(&resid[14]);
    if (stabpos  !=  0.) {
        b1cperr(p0);
        for (i = 0; i < 4; i++) {
            resid[14+i] = resid[14+i]+stabpos*p0[i];
        }
    }
    for (i = 0; i < 7; i++) {
        udot[i] = eudot[i];
    }
    for (i = 0; i < 4; i++) {
        mult[i] = emults[i];
    }
    b1crhs();
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   11 adds/subtracts/negates
                      4 multiplies
                      0 divides
                     29 assignments
*/
}

b1cmult(omults,owrank,omultmap)
    double omults[4];
    int *owrank,omultmap[4];
{
    int i;

    if (roustate != 3) {
        b1cseterr(34,24);
        return;
    }
    for (i = 0; i < 4; i++) {
        omults[i] = mult[i];
        if (i <= wrank-1) {
            omultmap[i] = multmap[i];
        } else {
            omultmap[i] = -1;
        }
    }
    *owrank = wrank;
}

b1creac(force,torque)
    double force[5][3],torque[5][3];
{
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

    if (roustate != 3) {
        b1cseterr(31,24);
        return;
    }
/*
Compute reaction forces for non-weld tree joints
*/
    ffkb[0][0] = (mfk[0][0]+ufk[0][0]);
    ffkb[0][1] = (mfk[0][1]+ufk[0][1]);
    ffkb[0][2] = (mfk[0][2]+ufk[0][2]);
    ffkb[1][0] = (mfk[1][0]+ufk[1][0]);
    ffkb[1][1] = (mfk[1][1]+ufk[1][1]);
    ffkb[1][2] = (mfk[1][2]+ufk[1][2]);
    ffkb[2][0] = (mfk[2][0]+ufk[2][0]);
    ffkb[2][1] = (mfk[2][1]+ufk[2][1]);
    ffkb[2][2] = (mfk[2][2]+ufk[2][2]);
    ffkb[3][0] = (mfk[3][0]+ufk[3][0]);
    ffkb[3][1] = (mfk[3][1]+ufk[3][1]);
    ffkb[3][2] = (mfk[3][2]+ufk[3][2]);
    ffkb[4][0] = (mfk[4][0]+ufk[4][0]);
    ffkb[4][1] = (mfk[4][1]+ufk[4][1]);
    ffkb[4][2] = (mfk[4][2]+ufk[4][2]);
    ttkb[0][0] = (mtk[0][0]+utk[0][0]);
    ttkb[0][1] = (mtk[0][1]+utk[0][1]);
    ttkb[0][2] = (mtk[0][2]+utk[0][2]);
    ttkb[1][0] = (mtk[1][0]+utk[1][0]);
    ttkb[1][1] = (mtk[1][1]+utk[1][1]);
    ttkb[1][2] = (mtk[1][2]+utk[1][2]);
    ttkb[2][0] = (mtk[2][0]+utk[2][0]);
    ttkb[2][1] = (mtk[2][1]+utk[2][1]);
    ttkb[2][2] = (mtk[2][2]+utk[2][2]);
    ttkb[3][0] = (mtk[3][0]+utk[3][0]);
    ttkb[3][1] = (mtk[3][1]+utk[3][1]);
    ttkb[3][2] = (mtk[3][2]+utk[3][2]);
    ttkb[4][0] = (mtk[4][0]+utk[4][0]);
    ttkb[4][1] = (mtk[4][1]+utk[4][1]);
    ttkb[4][2] = (mtk[4][2]+utk[4][2]);
    fc[6][0] = ((mk[4]*(AnkAtk[6][0]-gk[6][0]))-ffkb[4][0]);
    fc[6][1] = -ffkb[4][1];
    fc[6][2] = ((mk[4]*(AnkAtk[6][2]-gk[6][2]))-ffkb[4][2]);
    tc[6][0] = -(ttkb[4][0]-(fc[6][1]*rk[4][2]));
    tc[6][1] = ((ik[4][1][1]*Onkb[6][1])-(ttkb[4][1]+(fc[6][0]*rk[4][2])));
    tc[6][2] = -ttkb[4][2];
    fccikt[6][0] = ((fc[6][0]*c6)+(fc[6][2]*s6));
    fccikt[6][1] = fc[6][1];
    fccikt[6][2] = ((fc[6][2]*c6)-(fc[6][0]*s6));
    ffk[5][0] = (ffkb[3][0]-fccikt[6][0]);
    ffk[5][1] = (ffkb[3][1]-fccikt[6][1]);
    ffk[5][2] = (ffkb[3][2]-fccikt[6][2]);
    ttk[5][0] = (ttkb[3][0]-(((tc[6][0]*c6)+(tc[6][2]*s6))-(fccikt[6][1]*
      ri[4][2])));
    ttk[5][1] = (ttkb[3][1]-(tc[6][1]+(fccikt[6][0]*ri[4][2])));
    ttk[5][2] = (ttkb[3][2]-((tc[6][2]*c6)-(tc[6][0]*s6)));
    fc[5][0] = ((mk[3]*(AnkAtk[5][0]-gk[5][0]))-ffk[5][0]);
    fc[5][1] = -ffk[5][1];
    fc[5][2] = ((mk[3]*(AnkAtk[5][2]-gk[5][2]))-ffk[5][2]);
    tc[5][0] = -(ttk[5][0]-(fc[5][1]*rk[3][2]));
    tc[5][1] = ((ik[3][1][1]*Onkb[5][1])-(ttk[5][1]+(fc[5][0]*rk[3][2])));
    tc[5][2] = -ttk[5][2];
    fccikt[5][0] = ((fc[5][0]*c5)+(fc[5][2]*s5));
    fccikt[5][1] = fc[5][1];
    fccikt[5][2] = ((fc[5][2]*c5)-(fc[5][0]*s5));
    ffk[2][0] = (ffkb[0][0]-fccikt[5][0]);
    ffk[2][1] = (ffkb[0][1]-fccikt[5][1]);
    ffk[2][2] = (ffkb[0][2]-fccikt[5][2]);
    ttk[2][0] = (ttkb[0][0]-(((tc[5][0]*c5)+(tc[5][2]*s5))-(fccikt[5][1]*
      ri[3][2])));
    ttk[2][1] = (ttkb[0][1]-(tc[5][1]+(fccikt[5][0]*ri[3][2])));
    ttk[2][2] = (ttkb[0][2]-((tc[5][2]*c5)-(tc[5][0]*s5)));
    fc[4][0] = ((mk[2]*(AnkAtk[4][0]-gk[4][0]))-ffkb[2][0]);
    fc[4][1] = -ffkb[2][1];
    fc[4][2] = ((mk[2]*(AnkAtk[4][2]-gk[4][2]))-ffkb[2][2]);
    tc[4][0] = -(ttkb[2][0]-(fc[4][1]*rk[2][2]));
    tc[4][1] = ((ik[2][1][1]*Onkb[4][1])-(ttkb[2][1]+(fc[4][0]*rk[2][2])));
    tc[4][2] = -ttkb[2][2];
    fccikt[4][0] = ((fc[4][0]*c4)+(fc[4][2]*s4));
    fccikt[4][1] = fc[4][1];
    fccikt[4][2] = ((fc[4][2]*c4)-(fc[4][0]*s4));
    ffk[3][0] = (ffkb[1][0]-fccikt[4][0]);
    ffk[3][1] = (ffkb[1][1]-fccikt[4][1]);
    ffk[3][2] = (ffkb[1][2]-fccikt[4][2]);
    ttk[3][0] = (ttkb[1][0]-(((tc[4][0]*c4)+(tc[4][2]*s4))-(fccikt[4][1]*
      ri[2][2])));
    ttk[3][1] = (ttkb[1][1]-(tc[4][1]+(fccikt[4][0]*ri[2][2])));
    ttk[3][2] = (ttkb[1][2]-((tc[4][2]*c4)-(tc[4][0]*s4)));
    fc[3][0] = ((mk[1]*(AnkAtk[3][0]-gk[3][0]))-ffk[3][0]);
    fc[3][1] = -ffk[3][1];
    fc[3][2] = ((mk[1]*(AnkAtk[3][2]-gk[3][2]))-ffk[3][2]);
    tc[3][0] = -(ttk[3][0]-(fc[3][1]*rk[1][2]));
    tc[3][1] = ((ik[1][1][1]*Onkb[3][1])-(ttk[3][1]+(fc[3][0]*rk[1][2])));
    tc[3][2] = -ttk[3][2];
    fccikt[3][0] = ((fc[3][0]*c3)+(fc[3][2]*s3));
    fccikt[3][1] = fc[3][1];
    fccikt[3][2] = ((fc[3][2]*c3)-(fc[3][0]*s3));
    ffk[2][0] = (ffk[2][0]-fccikt[3][0]);
    ffk[2][1] = (ffk[2][1]-fccikt[3][1]);
    ffk[2][2] = (ffk[2][2]-fccikt[3][2]);
    ttk[2][0] = (ttk[2][0]-(((tc[3][0]*c3)+(tc[3][2]*s3))-(fccikt[3][1]*ri[1][2]
      )));
    ttk[2][1] = (ttk[2][1]-(tc[3][1]+(fccikt[3][0]*ri[1][2])));
    ttk[2][2] = (ttk[2][2]-((tc[3][2]*c3)-(tc[3][0]*s3)));
    fc[2][0] = ((mk[0]*(Ankb[2][0]-(9.81*s2)))-ffk[2][0]);
    fc[2][1] = -ffk[2][1];
    fc[2][2] = ((mk[0]*(AnkAtk[2][2]+(9.81*c2)))-ffk[2][2]);
    tc[2][0] = -(ttk[2][0]-(fc[2][1]*rk[0][2]));
    tc[2][1] = ((ik[0][1][1]*udot[2])-(ttk[2][1]+(fc[2][0]*rk[0][2])));
    tc[2][2] = -ttk[2][2];
    fccikt[2][0] = ((fc[2][0]*c2)+(fc[2][2]*s2));
    fccikt[2][1] = fc[2][1];
    fccikt[2][2] = ((fc[2][2]*c2)-(fc[2][0]*s2));
    ffk[1][0] = -fccikt[2][0];
    ffk[1][1] = -fccikt[2][1];
    ffk[1][2] = -fccikt[2][2];
    ttk[1][0] = -((tc[2][0]*c2)+(tc[2][2]*s2));
    ttk[1][1] = -tc[2][1];
    ttk[1][2] = -((tc[2][2]*c2)-(tc[2][0]*s2));
    fc[1][0] = -ffk[1][0];
    fc[1][1] = -ffk[1][1];
    fc[1][2] = -ffk[1][2];
    tc[1][0] = -ttk[1][0];
    tc[1][1] = -ttk[1][1];
    tc[1][2] = -ttk[1][2];
    fccikt[1][0] = fc[1][0];
    fccikt[1][1] = fc[1][1];
    fccikt[1][2] = fc[1][2];
    ffk[0][0] = -fccikt[1][0];
    ffk[0][1] = -fccikt[1][1];
    ffk[0][2] = -fccikt[1][2];
    ttk[0][0] = -(tc[1][0]-(fccikt[1][1]*q[1]));
    ttk[0][1] = -(tc[1][1]+(fccikt[1][0]*q[1]));
    ttk[0][2] = -tc[1][2];
    fc[0][0] = -ffk[0][0];
    fc[0][1] = -ffk[0][1];
    fc[0][2] = -ffk[0][2];
    tc[0][0] = -ttk[0][0];
    tc[0][1] = -ttk[0][1];
    tc[0][2] = -ttk[0][2];
    force[0][0] = fc[2][0];
    torque[0][0] = tc[2][0];
    force[0][1] = fc[2][1];
    torque[0][1] = tc[2][1];
    force[0][2] = fc[2][2];
    torque[0][2] = tc[2][2];
    force[1][0] = fc[3][0];
    torque[1][0] = tc[3][0];
    force[1][1] = fc[3][1];
    torque[1][1] = tc[3][1];
    force[1][2] = fc[3][2];
    torque[1][2] = tc[3][2];
    force[2][0] = fc[4][0];
    torque[2][0] = tc[4][0];
    force[2][1] = fc[4][1];
    torque[2][1] = tc[4][1];
    force[2][2] = fc[4][2];
    torque[2][2] = tc[4][2];
    force[3][0] = fc[5][0];
    torque[3][0] = tc[5][0];
    force[3][1] = fc[5][1];
    torque[3][1] = tc[5][1];
    force[3][2] = fc[5][2];
    torque[3][2] = tc[5][2];
    force[4][0] = fc[6][0];
    torque[4][0] = tc[6][0];
    force[4][1] = fc[6][1];
    torque[4][1] = tc[6][1];
    force[4][2] = fc[6][2];
    torque[4][2] = tc[6][2];
/*
Compute reaction forces for tree weld joints
*/
/*
 Used 0.05 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain  158 adds/subtracts/negates
                     77 multiplies
                      0 divides
                    156 assignments
*/
}

b1cmom(lm,am,ke)
    double lm[3],am[3],*ke;
{
/*
Compute system linear and angular momentum, and kinetic energy.

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
    double lk[5][3],hnk[5][3];

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(19,23);
        return;
    }
    lk[0][0] = (mk[0]*vnk[2][0]);
    lk[0][2] = (mk[0]*vnk[2][2]);
    lk[1][0] = (mk[1]*vnk[3][0]);
    lk[1][2] = (mk[1]*vnk[3][2]);
    lk[2][0] = (mk[2]*vnk[4][0]);
    lk[2][2] = (mk[2]*vnk[4][2]);
    lk[3][0] = (mk[3]*vnk[5][0]);
    lk[3][2] = (mk[3]*vnk[5][2]);
    lk[4][0] = (mk[4]*vnk[6][0]);
    lk[4][2] = (mk[4]*vnk[6][2]);
    hnk[0][1] = (ik[0][1][1]*u[2]);
    hnk[1][1] = (ik[1][1][1]*wk[3][1]);
    hnk[2][1] = (ik[2][1][1]*wk[4][1]);
    hnk[3][1] = (ik[3][1][1]*wk[5][1]);
    hnk[4][1] = (ik[4][1][1]*wk[6][1]);
    lm[0] = (lk[4][0]+(lk[3][0]+(lk[2][0]+(lk[0][0]+lk[1][0]))));
    lm[1] = 0.;
    lm[2] = (lk[4][2]+(lk[3][2]+(lk[2][2]+(lk[0][2]+lk[1][2]))));
    am[0] = 0.;
    am[1] = (((hnk[4][1]+((lk[4][0]*rnk[6][2])-(lk[4][2]*rnk[6][0])))+((
      hnk[3][1]+((lk[3][0]*rnk[5][2])-(lk[3][2]*rnk[5][0])))+((hnk[2][1]+((
      lk[2][0]*rnk[4][2])-(lk[2][2]*rnk[4][0])))+((hnk[0][1]+((lk[0][0]*
      rnk[2][2])-(lk[0][2]*rnk[2][0])))+(hnk[1][1]+((lk[1][0]*rnk[3][2])-(
      lk[1][2]*rnk[3][0])))))))-((com[2]*lm[0])-(com[0]*lm[2])));
    am[2] = 0.;
    *ke = (.5*(((hnk[4][1]*wk[6][1])+((lk[4][0]*vnk[6][0])+(lk[4][2]*vnk[6][2]))
      )+(((hnk[3][1]*wk[5][1])+((lk[3][0]*vnk[5][0])+(lk[3][2]*vnk[5][2])))+(((
      hnk[2][1]*wk[4][1])+((lk[2][0]*vnk[4][0])+(lk[2][2]*vnk[4][2])))+(((
      hnk[0][1]*u[2])+((lk[0][0]*vnk[2][0])+(lk[0][2]*vnk[2][2])))+((hnk[1][1]*
      wk[3][1])+((lk[1][0]*vnk[3][0])+(lk[1][2]*vnk[3][2]))))))));
/*
 Used 0.02 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain   38 adds/subtracts/negates
                     43 multiplies
                      0 divides
                     22 assignments
*/
}

b1csys(mtoto,cm,icm)
    double *mtoto,cm[3],icm[3][3];
{
/*
Compute system total mass, and instantaneous center of mass and
inertia matrix.

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
    double ikcnkt[7][3][3];

    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(20,23);
        return;
    }
    *mtoto = mtot;
    cm[0] = com[0];
    cm[1] = 0.;
    cm[2] = com[2];
    temp[0] = (((mk[3]*(rnk[5][2]*rnk[5][2]))+((cnk[5][0][0]*cnk[5][0][0])+(
      cnk[5][0][2]*cnk[5][0][2])))+(((mk[0]*(rnk[2][2]*rnk[2][2]))+((mk[1]*(
      rnk[3][2]*rnk[3][2]))+((cnk[3][0][0]*cnk[3][0][0])+(cnk[3][0][2]*
      cnk[3][0][2]))))+((mk[2]*(rnk[4][2]*rnk[4][2]))+((cnk[4][0][0]*
      cnk[4][0][0])+(cnk[4][0][2]*cnk[4][0][2])))));
    icm[0][0] = (1.+((((mk[4]*(rnk[6][2]*rnk[6][2]))+((cnk[6][0][0]*cnk[6][0][0]
      )+(cnk[6][0][2]*cnk[6][0][2])))+temp[0])-(mtot*(com[2]*com[2]))));
    icm[0][1] = 0.;
    temp[0] = ((((cnk[5][0][0]*cnk[5][2][0])+(cnk[5][0][2]*cnk[5][2][2]))-(mk[3]
      *(rnk[5][0]*rnk[5][2])))+((((cnk[4][0][0]*cnk[4][2][0])+(cnk[4][0][2]*
      cnk[4][2][2]))-(mk[2]*(rnk[4][0]*rnk[4][2])))+((((cnk[3][0][0]*
      cnk[3][2][0])+(cnk[3][0][2]*cnk[3][2][2]))-(mk[1]*(rnk[3][0]*rnk[3][2])))-
      (mk[0]*(rnk[2][0]*rnk[2][2])))));
    icm[0][2] = ((mtot*(com[0]*com[2]))+((((cnk[6][0][0]*cnk[6][2][0])+(
      cnk[6][0][2]*cnk[6][2][2]))-(mk[4]*(rnk[6][0]*rnk[6][2])))+temp[0]));
    icm[1][0] = icm[0][1];
    icm[1][1] = (((ik[4][1][1]+(mk[4]*((rnk[6][0]*rnk[6][0])+(rnk[6][2]*
      rnk[6][2]))))+((ik[3][1][1]+(mk[3]*((rnk[5][0]*rnk[5][0])+(rnk[5][2]*
      rnk[5][2]))))+((ik[2][1][1]+(mk[2]*((rnk[4][0]*rnk[4][0])+(rnk[4][2]*
      rnk[4][2]))))+((ik[0][1][1]+(mk[0]*((rnk[2][0]*rnk[2][0])+(rnk[2][2]*
      rnk[2][2]))))+(ik[1][1][1]+(mk[1]*((rnk[3][0]*rnk[3][0])+(rnk[3][2]*
      rnk[3][2]))))))))-(mtot*((com[0]*com[0])+(com[2]*com[2]))));
    icm[1][2] = 0.;
    icm[2][0] = icm[0][2];
    icm[2][1] = icm[1][2];
    temp[0] = (((mk[3]*(rnk[5][0]*rnk[5][0]))+((cnk[5][2][0]*cnk[5][2][0])+(
      cnk[5][2][2]*cnk[5][2][2])))+(((mk[0]*(rnk[2][0]*rnk[2][0]))+((mk[1]*(
      rnk[3][0]*rnk[3][0]))+((cnk[3][2][0]*cnk[3][2][0])+(cnk[3][2][2]*
      cnk[3][2][2]))))+((mk[2]*(rnk[4][0]*rnk[4][0]))+((cnk[4][2][0]*
      cnk[4][2][0])+(cnk[4][2][2]*cnk[4][2][2])))));
    icm[2][2] = (1.+((((mk[4]*(rnk[6][0]*rnk[6][0]))+((cnk[6][2][0]*cnk[6][2][0]
      )+(cnk[6][2][2]*cnk[6][2][2])))+temp[0])-(mtot*(com[0]*com[0]))));
/*
 Used 0.05 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain   57 adds/subtracts/negates
                     78 multiplies
                      0 divides
                     16 assignments
*/
}

b1cpos(body,pt,loc)
    int body;
    double pt[3],loc[3];
{
/*
Return inertial frame location of a point on a body.

*/
    double pv[3];

    if (b1cchkbnum(21,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(21,23);
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

b1cvel(body,pt,velo)
    int body;
    double pt[3],velo[3];
{
/*
Return inertial frame velocity of a point on a body.

*/
    double pv[3];

    if (b1cchkbnum(22,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(22,23);
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

b1corient(body,dircos)
    int body;
    double dircos[3][3];
{
/*
Return orientation of body w.r.t. ground frame.

*/

    if (b1cchkbnum(23,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(23,23);
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

b1cangvel(body,avel)
    int body;
    double avel[3];
{
/*
Return angular velocity of the body.

*/

    if (b1cchkbnum(24,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(24,23);
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

b1ctrans(frbod,ivec,tobod,ovec)
    int frbod;
    double ivec[3];
    int tobod;
    double ovec[3];
{
/*
Transform ivec from frbod frame to tobod frame.

*/
    double pv[3];

    if (b1cchkbnum(25,frbod) != 0) {
        return;
    }
    if (b1cchkbnum(25,tobod) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(25,23);
        return;
    }
    if (frbod == tobod) {
        b1cvcopy(ivec,ovec);
        return;
    }
    if (frbod == -1) {
        b1cvcopy(ivec,pv);
        ovec[0] = pv[0]*cnb[tobod][0][0]+pv[1]*cnb[tobod][1][0]+pv[2]*cnb[tobod
          ][2][0];
        ovec[1] = pv[0]*cnb[tobod][0][1]+pv[1]*cnb[tobod][1][1]+pv[2]*cnb[tobod
          ][2][1];
        ovec[2] = pv[0]*cnb[tobod][0][2]+pv[1]*cnb[tobod][1][2]+pv[2]*cnb[tobod
          ][2][2];
        return;
    }
    if (tobod == -1) {
        b1cvcopy(ivec,pv);
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

b1crel2cart(coord,body,point,linchg,rotchg)
    int coord,body;
    double point[3],linchg[3],rotchg[3];
{
/* Return derivative of pt loc and body orient w.r.t. hinge rate
*/
    int x,i,gnd;
    double lin[3],pv[3];

    if ((coord < 0) || (coord > 6)) {
        b1cseterr(59,45);
        return;
    }
    if (b1cchkbnum(59,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1cseterr(59,23);
        return;
    }
    gnd = -1;
    if (body == gnd) {
        x = -1;
    } else {
        x = firstq[body]+njntdof[body]-1;
    }
    if (x < coord) {
        b1cvset(0.,0.,0.,linchg);
        b1cvset(0.,0.,0.,rotchg);
        return;
    }
    b1cdovpk();
    for (i = 0; i < 3; i++) {
        rotchg[i] = Wpk[coord][x][i];
        lin[i] = Vpk[coord][x][i];
    }
    if (body == gnd) {
        b1cvcopy(point,pv);
    } else {
        pv[0] = rcom[body][0]+point[0];
        pv[1] = rcom[body][1]+point[1];
        pv[2] = rcom[body][2]+point[2];
    }
    b1cvcross(rotchg,pv,linchg);
    b1cvadd(linchg,lin,linchg);
}

b1cacc(body,pt,accel)
    int body;
    double pt[3],accel[3];
{
/*
Return linear acceleration a point of the specified body.

*/
    double pv[3];

    if (b1cchkbnum(32,body) != 0) {
        return;
    }
    if (roustate != 3) {
        b1cseterr(32,24);
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

b1cangacc(body,aacc)
    int body;
    double aacc[3];
{
/*
Return angular acceleration of the body.

*/

    if (b1cchkbnum(33,body) != 0) {
        return;
    }
    if (roustate != 3) {
        b1cseterr(33,24);
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

b1cgrav(gravin)
    double gravin[3];
{

    b1cseterr(1,19);
    roustate = 0;
}

b1cmass(body,massin)
    int body;
    double massin;
{

    if (b1cchkbnum(2,body) != 0) {
        return;
    }
    if (body == -1) {
        b1cseterr(2,15);
        return;
    }
    if (mkq[body] != 0) {
        mk[body] = massin;
        mkq[body] = 3;
    } else {
        b1cseterr(2,19);
    }
    roustate = 0;
}

b1ciner(body,inerin)
    int body;
    double inerin[3][3];
{
    int anyques;

    if (b1cchkbnum(3,body) != 0) {
        return;
    }
    if (body == -1) {
        b1cseterr(3,15);
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
        b1cseterr(3,19);
    }
    roustate = 0;
}

b1cbtj(joint,btjin)
    int joint;
    double btjin[3];
{
    int anyques;

    if (b1cchkjnum(4,joint) != 0) {
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
        b1cseterr(4,19);
    }
    roustate = 0;
}

b1citj(joint,itjin)
    int joint;
    double itjin[3];
{
    int anyques;

    if (b1cchkjnum(5,joint) != 0) {
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
        b1cseterr(5,19);
    }
    roustate = 0;
}

b1cpin(joint,pinno,pinin)
    int joint,pinno;
    double pinin[3];
{
    int anyques,offs;

    if (b1cchkjpin(6,joint,pinno) != 0) {
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
        b1cseterr(6,19);
    }
    roustate = 0;
}

b1cpres(joint,axis,presin)
    int joint,axis,presin;
{
    int anyques;

    if (b1cchkjaxis(37,joint,axis) != 0) {
        return;
    }
    if ((presin != 0) && (presin != 1)) {
        b1cseterr(37,20);
    }
    anyques = 0;
    if (presq[b1cindx(joint,axis)]  !=  0) {
        if (presin  !=  0) {
            pres[b1cindx(joint,axis)] = 1.;
        } else {
            pres[b1cindx(joint,axis)] = 0.;
        }
        presq[b1cindx(joint,axis)] = 3;
        anyques = 1;
    }
    if (anyques == 0) {
        b1cseterr(37,19);
    }
    wwflg = 0;
}

b1cstab(velin,posin)
    double velin,posin;
{

    stabvel = velin;
    stabvelq = 3;
    stabpos = posin;
    stabposq = 3;
}

b1cgetgrav(gravout)
    double gravout[3];
{

    gravout[0] = grav[0];
    gravout[1] = grav[1];
    gravout[2] = grav[2];
}

b1cgetmass(body,massout)
    int body;
    double *massout;
{

    if (b1cchkbnum(40,body) != 0) {
        return;
    }
    if (body == -1) {
        b1cseterr(40,15);
        return;
    }
    *massout = mk[body];
}

b1cgetiner(body,inerout)
    int body;
    double inerout[3][3];
{

    if (b1cchkbnum(41,body) != 0) {
        return;
    }
    if (body == -1) {
        b1cseterr(41,15);
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

b1cgetbtj(joint,btjout)
    int joint;
    double btjout[3];
{

    if (b1cchkjnum(42,joint) != 0) {
        return;
    }
    btjout[0] = rk[joint][0];
    btjout[1] = rk[joint][1];
    btjout[2] = rk[joint][2];
}

b1cgetitj(joint,itjout)
    int joint;
    double itjout[3];
{

    if (b1cchkjnum(43,joint) != 0) {
        return;
    }
    itjout[0] = ri[joint][0];
    itjout[1] = ri[joint][1];
    itjout[2] = ri[joint][2];
}

b1cgetpin(joint,pinno,pinout)
    int joint,pinno;
    double pinout[3];
{
    int offs;

    if (b1cchkjpin(44,joint,pinno) != 0) {
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

b1cgetpres(joint,axis,presout)
    int joint,axis,*presout;
{

    if (b1cchkjaxis(45,joint,axis) != 0) {
        return;
    }
    if (pres[b1cindx(joint,axis)]  !=  0.) {
        *presout = 1;
    } else {
        *presout = 0;
    }
}

b1cgetstab(velout,posout)
    double *velout,*posout;
{

    *velout = stabvel;
    *posout = stabpos;
}

b1cinfo(info)
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
    info[11] = 4;
/* info entries from 12-49 are reserved */
}

b1cjnt(joint,info,tran)
    int joint,info[50],tran[6];
{
    int i,offs;

    if (b1cchkjnum(48,joint) != 0) {
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

b1ccons(consno,info)
    int consno,info[50];
{

    if (b1cchkucnum(49,consno) != 0) {
        return;
    }
    info[0] = 1;
    info[1] = firstu[consno];
/* info entries from 2-49 are reserved */
}
int 
b1cchkbnum(routine,bnum)
    int routine,bnum;
{

    if ((bnum < -1) || (bnum > 4)) {
        b1cseterr(routine,15);
        return 1;
    }
    return 0;
}
int 
b1cchkjnum(routine,jnum)
    int routine,jnum;
{

    if ((jnum < 0) || (jnum > 4)) {
        b1cseterr(routine,16);
        return 1;
    }
    return 0;
}
int 
b1cchkucnum(routine,ucnum)
    int routine,ucnum;
{

    if ((ucnum < 0) || (ucnum > 3)) {
        b1cseterr(routine,21);
        return 1;
    }
    return 0;
}
int 
b1cchkjaxis(routine,jnum,axnum)
    int routine,jnum,axnum;
{
    int maxax;

    if (b1cchkjnum(routine,jnum) != 0) {
        return 1;
    }
    if ((axnum < 0) || (axnum > 6)) {
        b1cseterr(routine,17);
        return 1;
    }
    maxax = njntdof[jnum]-1;
    if ((jtype[jnum] == 4) || (jtype[jnum] == 6) || (jtype[jnum] == 21)) {
        maxax = maxax+1;
    }
    if (axnum > maxax) {
        b1cseterr(routine,18);
        return 1;
    }
    return 0;
}
int 
b1cchkjpin(routine,jnum,pinno)
    int routine,jnum,pinno;
{
    int maxax,pinok;

    if (b1cchkjnum(routine,jnum) != 0) {
        return 1;
    }
    if ((pinno < 0) || (pinno > 5)) {
        b1cseterr(routine,17);
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
        b1cseterr(routine,18);
        return 1;
    }
    return 0;
}

/* Convert state to form using 1-2-3 Euler angles for ball joints. */

b1cst2ang(st,stang)
    double st[7],stang[7];
{
    int i;

    for (i = 0; i < 7; i++) {
        stang[i] = st[i];
    }
}

/* Convert 1-2-3 form of state back to Euler parameters for ball joints. */

b1cang2st(stang,st)
    double stang[7],st[7];
{
    int i;

    for (i = 0; i < 7; i++) {
        st[i] = stang[i];
    }
}

/* Normalize Euler parameters in state. */

b1cnrmsterr(st,normst,routine)
    double st[7],normst[7];
    int routine;
{
    int i;

    for (i = 0; i < 7; i++) {
        normst[i] = st[i];
    }
}

b1cnormst(st,normst)
    double st[7],normst[7];
{

    b1cnrmsterr(st,normst,0);
}

b1cgentime(gentm)
    int *gentm;
{

    *gentm = 103203;
}
/*
Done. CPU seconds used: 0.63  Memory used: 966656 bytes.
Equation complexity:
  sdstate:    88 adds   127 multiplies     2 divides   226 assignments
  sdderiv:  1034 adds  1042 multiplies    36 divides  1589 assignments
  sdresid:   264 adds   193 multiplies     0 divides   305 assignments
  sdreac:    158 adds    77 multiplies     0 divides   156 assignments
  sdmom:      38 adds    43 multiplies     0 divides    22 assignments
  sdsys:      57 adds    78 multiplies     0 divides    16 assignments
*/
