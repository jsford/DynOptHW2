/*
Generated 21-Sep-2004 10:34:48 by SD/FAST, Kane's formulation
(sdfast B.2.6 #70405) on machine ID 690c54f5
Copyright (c) 1990-1996 Symbolic Dynamics, Inc.
Copyright (c) 1990-1996 Parametric Technology Corp.
RESTRICTED RIGHTS LEGEND: Use, duplication, or disclosure by the U.S.
Government is subject to restrictions as set forth in subparagraph
(c)(1)(ii) of the Rights in Technical Data and Computer Software
clause at DFARS 52.227-7013 and similar clauses in the FAR and NASA
FAR Supplement.  Symbolic Dynamics, Inc., Mountain View, CA 94041


ROADMAP (b1ds.sd)

Bodies        Inb
No  Name      body Joint type  Coords q         Multipliers
--- --------- ---- ----------- ---------------- -----------------------
 -1 $ground                                    |
  0 l.shin     -1  Pin           0             |
  1 l.thigh     0  Pin           1             |
  2 pelvis      1  Pin           2             |
  3 r.thigh     2  Pin           3             |
  4 r.shin      3  Pin           4             |

Loop Joints                    Pseudo Coords lq

  5 r.shin     -1  Pin           0             |  0   1   2   3   4 

*/
#include <math.h>
#include <stdio.h>

typedef struct {
    int ground_,nbod_,ndof_,ncons_,nloop_,nldof_,nloopc_,nball_,nlball_,npres_,
      nuser_;
    int jtype_[6],inb_[6],outb_[6],njntdof_[6],njntc_[6],njntp_[6],firstq_[6],
      ballq_[6],firstm_[6],firstp_[6];
    int trans_[6];
} b1dsgtopo_t;
#define ground (b1dsgtopo.ground_)
#define nbod (b1dsgtopo.nbod_)
#define ndof (b1dsgtopo.ndof_)
#define ncons (b1dsgtopo.ncons_)
#define nloop (b1dsgtopo.nloop_)
#define nldof (b1dsgtopo.nldof_)
#define nloopc (b1dsgtopo.nloopc_)
#define nball (b1dsgtopo.nball_)
#define nlball (b1dsgtopo.nlball_)
#define npres (b1dsgtopo.npres_)
#define nuser (b1dsgtopo.nuser_)
#define jtype (b1dsgtopo.jtype_)
#define inb (b1dsgtopo.inb_)
#define outb (b1dsgtopo.outb_)
#define njntdof (b1dsgtopo.njntdof_)
#define njntc (b1dsgtopo.njntc_)
#define njntp (b1dsgtopo.njntp_)
#define firstq (b1dsgtopo.firstq_)
#define ballq (b1dsgtopo.ballq_)
#define firstm (b1dsgtopo.firstm_)
#define firstp (b1dsgtopo.firstp_)
#define trans (b1dsgtopo.trans_)

typedef struct {
    double grav_[3],mk_[5],ik_[5][3][3],pin_[5][3];
    double rk_[5][3],ri_[5][3],pres_[5],stabvel_,stabpos_;
    double inbpin1_[1][3],inbpin2_[1][3],inbpin3_[1][3],inbref_[1][3];
    double bodypin_[1][3],bodyref_[1][3],lbtj_[1][3],litj_[1][3],lpres_[1];
    int mfrcflg_,roustate_,vpkflg_,inerflg_,mmflg_,mmlduflg_,wwflg_,ltauflg_,
      fs0flg_,ii_,mmap_[5];
    int gravq_[3],mkq_[5],ikq_[5][3][3],pinq_[5][3],rkq_[5][3],riq_[5][3],presq_
      [5],stabvelq_,stabposq_;
    int inbpin1q_[1][3],inbpin2q_[1][3],inbpin3q_[1][3],inbrefq_[1][3],bodypinq_
      [1][3],bodyrefq_[1][3],lbtjq_[1][3],litjq_[1][3],lpresq_[1];
    double mtot_,psmkg_,rhead_[5][3],rcom_[5][3],mkrcomt_[5][3][3],psikg_[3][3],
      psrcomg_[3],psrkg_[3],psrig_[3],psmk_[5],psik_[5][3][3],psrcom_[5][3],
      psrk_[5][3],psri_[5][3];
    double ipin_[1][3],ipin2_[1][3],iref_[1][3],iperp_[1][3],opin_[1][3],oref_[1
      ][3],operp_[1][3],ghand_[1];
} b1dsginput_t;
#define grav (b1dsginput.grav_)
#define mk (b1dsginput.mk_)
#define ik (b1dsginput.ik_)
#define pin (b1dsginput.pin_)
#define rk (b1dsginput.rk_)
#define ri (b1dsginput.ri_)
#define pres (b1dsginput.pres_)
#define stabvel (b1dsginput.stabvel_)
#define stabpos (b1dsginput.stabpos_)
#define rhead (b1dsginput.rhead_)
#define rcom (b1dsginput.rcom_)
#define psrcomg (b1dsginput.psrcomg_)
#define psrcom (b1dsginput.psrcom_)
#define mkrcomt (b1dsginput.mkrcomt_)
#define psmk (b1dsginput.psmk_)
#define psik (b1dsginput.psik_)
#define psrk (b1dsginput.psrk_)
#define psri (b1dsginput.psri_)
#define psmkg (b1dsginput.psmkg_)
#define psikg (b1dsginput.psikg_)
#define psrkg (b1dsginput.psrkg_)
#define psrig (b1dsginput.psrig_)
#define mtot (b1dsginput.mtot_)
#define inbpin1 (b1dsginput.inbpin1_)
#define inbpin2 (b1dsginput.inbpin2_)
#define inbpin3 (b1dsginput.inbpin3_)
#define inbref (b1dsginput.inbref_)
#define bodypin (b1dsginput.bodypin_)
#define bodyref (b1dsginput.bodyref_)
#define lbtj (b1dsginput.lbtj_)
#define litj (b1dsginput.litj_)
#define lpres (b1dsginput.lpres_)
#define ipin (b1dsginput.ipin_)
#define ipin2 (b1dsginput.ipin2_)
#define iref (b1dsginput.iref_)
#define iperp (b1dsginput.iperp_)
#define opin (b1dsginput.opin_)
#define oref (b1dsginput.oref_)
#define operp (b1dsginput.operp_)
#define ghand (b1dsginput.ghand_)
#define mfrcflg (b1dsginput.mfrcflg_)
#define roustate (b1dsginput.roustate_)
#define vpkflg (b1dsginput.vpkflg_)
#define inerflg (b1dsginput.inerflg_)
#define mmflg (b1dsginput.mmflg_)
#define mmlduflg (b1dsginput.mmlduflg_)
#define wwflg (b1dsginput.wwflg_)
#define ltauflg (b1dsginput.ltauflg_)
#define fs0flg (b1dsginput.fs0flg_)
#define ii (b1dsginput.ii_)
#define mmap (b1dsginput.mmap_)
#define gravq (b1dsginput.gravq_)
#define mkq (b1dsginput.mkq_)
#define ikq (b1dsginput.ikq_)
#define pinq (b1dsginput.pinq_)
#define rkq (b1dsginput.rkq_)
#define riq (b1dsginput.riq_)
#define presq (b1dsginput.presq_)
#define stabvelq (b1dsginput.stabvelq_)
#define stabposq (b1dsginput.stabposq_)
#define lbtjq (b1dsginput.lbtjq_)
#define litjq (b1dsginput.litjq_)
#define lpresq (b1dsginput.lpresq_)
#define inbpin1q (b1dsginput.inbpin1q_)
#define inbpin2q (b1dsginput.inbpin2q_)
#define inbpin3q (b1dsginput.inbpin3q_)
#define inbrefq (b1dsginput.inbrefq_)
#define bodypinq (b1dsginput.bodypinq_)
#define bodyrefq (b1dsginput.bodyrefq_)

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
    double perr_[5],verr_[5],aerr_[5],mult_[5],ufk_[5][3],utk_[5][3],mfk_[5][3],
      mtk_[5][3];
    double utau_[5],mtau_[5],uacc_[5],uvel_[5],upos_[5];
    double Cio_[1][3][3],Cibo_[1][3][3],Cibob_[1][3][3],eul1_[1],eul2_[1],eul3_[
      1],eul4_[1];
    double eul1dot_[1],eul2dot_[1],eul3dot_[1],eul4dot_[1],eul1a_[1],eul2a_[1],
      eul3a_[1];
    double ceul1_[1],ceul2_[1],ceul3_[1],seul1_[1],seul2_[1],seul3_[1],sli1_[1],
      sli2_[1],sli3_[1];
    double sli1v_[1],sli2v_[1],sli3v_[1],sli1a_[1],sli2a_[1],sli3a_[1],ipin2x_[1
      ][3];
    double ltau_[1],mltau_[1],lacc_[1],lvel_[1],lpos_[1],lq_[1];
    double mltaufi_[1][3],mltaufo_[1][3],mltauti_[1][3],mltauto_[1][3],Woio_[1][
      3],Woiob_[1][3],Ooiob_[1][3];
    double vt1_[1][3],vt2_[1][3],vt2a_[1][3],vt3_[1][3],vt4_[1][3],vt5_[1][3],
      vt6_[1][3];
    double vt7_[1][3],vt8_[1][3],vt9_[1][3],vt10_[1][3],vt10a_[1][3],vt10b_[1][3
      ],vt10c_[1][3];
    double vt11_[1][3],vt12_[1][3],vt13_[1][3],vt13a_[1][3],vt14_[1][3],vt15_[1
      ][3];
    double vt16_[1][3],vt17_[1][3],vt18_[1][3],vt19_[1][3],vt20_[1][3],vt21_[1][
      3];
    double vt22_[1][3],vt23_[1][3],vt24_[1][3],vt25_[1][3],vt26_[1][3],vt26a_[1
      ][3],vt27_[1][3];
    double s0_,c0_,s1_,c1_,s2_,c2_,s3_,c3_,s4_,c4_;
} b1dsgstate_t;
#define curtim (b1dsgstate.curtim_)
#define q (b1dsgstate.q_)
#define qn (b1dsgstate.qn_)
#define u (b1dsgstate.u_)
#define cnk (b1dsgstate.cnk_)
#define cnb (b1dsgstate.cnb_)
#define rnkg (b1dsgstate.rnkg_)
#define rnk (b1dsgstate.rnk_)
#define rnb (b1dsgstate.rnb_)
#define vnk (b1dsgstate.vnk_)
#define vnb (b1dsgstate.vnb_)
#define wk (b1dsgstate.wk_)
#define wb (b1dsgstate.wb_)
#define com (b1dsgstate.com_)
#define Cik (b1dsgstate.Cik_)
#define Cib (b1dsgstate.Cib_)
#define rikt (b1dsgstate.rikt_)
#define Iko (b1dsgstate.Iko_)
#define mkrk (b1dsgstate.mkrk_)
#define Wkk (b1dsgstate.Wkk_)
#define Vkk (b1dsgstate.Vkk_)
#define dik (b1dsgstate.dik_)
#define rpp (b1dsgstate.rpp_)
#define rpk (b1dsgstate.rpk_)
#define rik (b1dsgstate.rik_)
#define rik2 (b1dsgstate.rik2_)
#define rpri (b1dsgstate.rpri_)
#define Wik (b1dsgstate.Wik_)
#define Vik (b1dsgstate.Vik_)
#define Wirk (b1dsgstate.Wirk_)
#define rkWkk (b1dsgstate.rkWkk_)
#define Wkrpk (b1dsgstate.Wkrpk_)
#define VikWkr (b1dsgstate.VikWkr_)
#define wbrcom (b1dsgstate.wbrcom_)
#define perr (b1dsgstate.perr_)
#define verr (b1dsgstate.verr_)
#define aerr (b1dsgstate.aerr_)
#define mult (b1dsgstate.mult_)
#define ufk (b1dsgstate.ufk_)
#define utk (b1dsgstate.utk_)
#define utau (b1dsgstate.utau_)
#define mfk (b1dsgstate.mfk_)
#define mtk (b1dsgstate.mtk_)
#define mtau (b1dsgstate.mtau_)
#define uacc (b1dsgstate.uacc_)
#define uvel (b1dsgstate.uvel_)
#define upos (b1dsgstate.upos_)
#define Cio (b1dsgstate.Cio_)
#define Cibo (b1dsgstate.Cibo_)
#define Cibob (b1dsgstate.Cibob_)
#define ltau (b1dsgstate.ltau_)
#define mltau (b1dsgstate.mltau_)
#define lacc (b1dsgstate.lacc_)
#define lvel (b1dsgstate.lvel_)
#define lpos (b1dsgstate.lpos_)
#define lq (b1dsgstate.lq_)
#define mltaufi (b1dsgstate.mltaufi_)
#define mltaufo (b1dsgstate.mltaufo_)
#define mltauti (b1dsgstate.mltauti_)
#define mltauto (b1dsgstate.mltauto_)
#define Woio (b1dsgstate.Woio_)
#define Woiob (b1dsgstate.Woiob_)
#define Ooiob (b1dsgstate.Ooiob_)
#define eul1 (b1dsgstate.eul1_)
#define eul2 (b1dsgstate.eul2_)
#define eul3 (b1dsgstate.eul3_)
#define eul4 (b1dsgstate.eul4_)
#define eul1dot (b1dsgstate.eul1dot_)
#define eul2dot (b1dsgstate.eul2dot_)
#define eul3dot (b1dsgstate.eul3dot_)
#define eul4dot (b1dsgstate.eul4dot_)
#define eul1a (b1dsgstate.eul1a_)
#define eul2a (b1dsgstate.eul2a_)
#define eul3a (b1dsgstate.eul3a_)
#define ceul1 (b1dsgstate.ceul1_)
#define ceul2 (b1dsgstate.ceul2_)
#define ceul3 (b1dsgstate.ceul3_)
#define seul1 (b1dsgstate.seul1_)
#define seul2 (b1dsgstate.seul2_)
#define seul3 (b1dsgstate.seul3_)
#define sli1 (b1dsgstate.sli1_)
#define sli2 (b1dsgstate.sli2_)
#define sli3 (b1dsgstate.sli3_)
#define sli1v (b1dsgstate.sli1v_)
#define sli2v (b1dsgstate.sli2v_)
#define sli3v (b1dsgstate.sli3v_)
#define sli1a (b1dsgstate.sli1a_)
#define sli2a (b1dsgstate.sli2a_)
#define sli3a (b1dsgstate.sli3a_)
#define ipin2x (b1dsgstate.ipin2x_)
#define vt1 (b1dsgstate.vt1_)
#define vt2 (b1dsgstate.vt2_)
#define vt2a (b1dsgstate.vt2a_)
#define vt3 (b1dsgstate.vt3_)
#define vt4 (b1dsgstate.vt4_)
#define vt5 (b1dsgstate.vt5_)
#define vt6 (b1dsgstate.vt6_)
#define vt7 (b1dsgstate.vt7_)
#define vt8 (b1dsgstate.vt8_)
#define vt9 (b1dsgstate.vt9_)
#define vt10 (b1dsgstate.vt10_)
#define vt10a (b1dsgstate.vt10a_)
#define vt10b (b1dsgstate.vt10b_)
#define vt10c (b1dsgstate.vt10c_)
#define vt11 (b1dsgstate.vt11_)
#define vt12 (b1dsgstate.vt12_)
#define vt13 (b1dsgstate.vt13_)
#define vt13a (b1dsgstate.vt13a_)
#define vt14 (b1dsgstate.vt14_)
#define vt15 (b1dsgstate.vt15_)
#define vt16 (b1dsgstate.vt16_)
#define vt17 (b1dsgstate.vt17_)
#define vt18 (b1dsgstate.vt18_)
#define vt19 (b1dsgstate.vt19_)
#define vt20 (b1dsgstate.vt20_)
#define vt21 (b1dsgstate.vt21_)
#define vt22 (b1dsgstate.vt22_)
#define vt23 (b1dsgstate.vt23_)
#define vt24 (b1dsgstate.vt24_)
#define vt25 (b1dsgstate.vt25_)
#define vt26 (b1dsgstate.vt26_)
#define vt26a (b1dsgstate.vt26a_)
#define vt27 (b1dsgstate.vt27_)
#define s0 (b1dsgstate.s0_)
#define c0 (b1dsgstate.c0_)
#define s1 (b1dsgstate.s1_)
#define c1 (b1dsgstate.c1_)
#define s2 (b1dsgstate.s2_)
#define c2 (b1dsgstate.c2_)
#define s3 (b1dsgstate.s3_)
#define c3 (b1dsgstate.c3_)
#define s4 (b1dsgstate.s4_)
#define c4 (b1dsgstate.c4_)

typedef struct {
    double fs0_[5],qdot_[5],Otk_[5][3],Atk_[5][3],AiOiWi_[5][3],Fstar_[5][3];
    double Tstar_[5][3],Fstark_[5][3],Tstark_[5][3],IkWk_[5][3],WkIkWk_[5][3],
      gk_[5][3],IkbWk_[5][3],WkIkbWk_[5][3];
    double w0w0_[5],w1w1_[5],w2w2_[5],w0w1_[5],w0w2_[5],w1w2_[5];
    double w00w11_[5],w00w22_[5],w11w22_[5],ww_[5][5],qraux_[5];
    double mm_[5][5],mlo_[5][5],mdi_[5],IkWpk_[5][5][3],works_[5],workss_[5][5];
    double Wpk_[5][5][3],Vpk_[5][5][3],VWri_[5][5][3];
    double ltaufk_[5][3],ltautk_[5][3],ltauc_[1],ltci_[1][3],ltc_[1][3],lfci_[1
      ][3],lfc_[1][3];
    double Tinb_[1][3],Toutb_[1][3],ltaufi_[1][3],ltaufo_[1][3],ltauti_[1][3],
      ltauto_[1][3];
    int wmap_[5],multmap_[5],jpvt_[5],wsiz_,wrank_;
} b1dsglhs_t;
#define qdot (b1dsglhs.qdot_)
#define Otk (b1dsglhs.Otk_)
#define Atk (b1dsglhs.Atk_)
#define AiOiWi (b1dsglhs.AiOiWi_)
#define Fstar (b1dsglhs.Fstar_)
#define Tstar (b1dsglhs.Tstar_)
#define fs0 (b1dsglhs.fs0_)
#define Fstark (b1dsglhs.Fstark_)
#define Tstark (b1dsglhs.Tstark_)
#define IkWk (b1dsglhs.IkWk_)
#define IkbWk (b1dsglhs.IkbWk_)
#define WkIkWk (b1dsglhs.WkIkWk_)
#define WkIkbWk (b1dsglhs.WkIkbWk_)
#define gk (b1dsglhs.gk_)
#define w0w0 (b1dsglhs.w0w0_)
#define w1w1 (b1dsglhs.w1w1_)
#define w2w2 (b1dsglhs.w2w2_)
#define w0w1 (b1dsglhs.w0w1_)
#define w0w2 (b1dsglhs.w0w2_)
#define w1w2 (b1dsglhs.w1w2_)
#define w00w11 (b1dsglhs.w00w11_)
#define w00w22 (b1dsglhs.w00w22_)
#define w11w22 (b1dsglhs.w11w22_)
#define ww (b1dsglhs.ww_)
#define qraux (b1dsglhs.qraux_)
#define ltaufk (b1dsglhs.ltaufk_)
#define ltautk (b1dsglhs.ltautk_)
#define ltci (b1dsglhs.ltci_)
#define ltc (b1dsglhs.ltc_)
#define lfci (b1dsglhs.lfci_)
#define lfc (b1dsglhs.lfc_)
#define Tinb (b1dsglhs.Tinb_)
#define Toutb (b1dsglhs.Toutb_)
#define ltauc (b1dsglhs.ltauc_)
#define ltaufi (b1dsglhs.ltaufi_)
#define ltaufo (b1dsglhs.ltaufo_)
#define ltauti (b1dsglhs.ltauti_)
#define ltauto (b1dsglhs.ltauto_)
#define mm (b1dsglhs.mm_)
#define mlo (b1dsglhs.mlo_)
#define mdi (b1dsglhs.mdi_)
#define IkWpk (b1dsglhs.IkWpk_)
#define works (b1dsglhs.works_)
#define workss (b1dsglhs.workss_)
#define Wpk (b1dsglhs.Wpk_)
#define Vpk (b1dsglhs.Vpk_)
#define VWri (b1dsglhs.VWri_)
#define wmap (b1dsglhs.wmap_)
#define multmap (b1dsglhs.multmap_)
#define jpvt (b1dsglhs.jpvt_)
#define wsiz (b1dsglhs.wsiz_)
#define wrank (b1dsglhs.wrank_)

typedef struct {
    double fs_[5],udot_[5],tauc_[5],dyad_[5][3][3],fc_[5][3],tc_[5][3];
    double ank_[5][3],onk_[5][3],Onkb_[5][3],AOnkri_[5][3],Ankb_[5][3],AnkAtk_[5
      ][3],anb_[5][3],onb_[5][3],dyrcom_[5][3];
    double ffk_[5][3],ttk_[5][3],fccikt_[5][3],ffkb_[5][3],ttkb_[5][3];
} b1dsgrhs_t;
#define fs (b1dsgrhs.fs_)
#define udot (b1dsgrhs.udot_)
#define ank (b1dsgrhs.ank_)
#define anb (b1dsgrhs.anb_)
#define onk (b1dsgrhs.onk_)
#define onb (b1dsgrhs.onb_)
#define Onkb (b1dsgrhs.Onkb_)
#define AOnkri (b1dsgrhs.AOnkri_)
#define Ankb (b1dsgrhs.Ankb_)
#define AnkAtk (b1dsgrhs.AnkAtk_)
#define dyrcom (b1dsgrhs.dyrcom_)
#define ffk (b1dsgrhs.ffk_)
#define ttk (b1dsgrhs.ttk_)
#define fccikt (b1dsgrhs.fccikt_)
#define ffkb (b1dsgrhs.ffkb_)
#define ttkb (b1dsgrhs.ttkb_)
#define dyad (b1dsgrhs.dyad_)
#define fc (b1dsgrhs.fc_)
#define tc (b1dsgrhs.tc_)
#define tauc (b1dsgrhs.tauc_)

typedef struct {
    double temp_[3000],tmat1_[3][3],tmat2_[3][3],tvec1_[3],tvec2_[3],tvec3_[3],
      tvec4_[3],tvec5_[3];
    double tsc1_,tsc2_,tsc3_;
} b1dsgtemp_t;
#define temp (b1dsgtemp.temp_)
#define tmat1 (b1dsgtemp.tmat1_)
#define tmat2 (b1dsgtemp.tmat2_)
#define tvec1 (b1dsgtemp.tvec1_)
#define tvec2 (b1dsgtemp.tvec2_)
#define tvec3 (b1dsgtemp.tvec3_)
#define tvec4 (b1dsgtemp.tvec4_)
#define tvec5 (b1dsgtemp.tvec5_)
#define tsc1 (b1dsgtemp.tsc1_)
#define tsc2 (b1dsgtemp.tsc2_)
#define tsc3 (b1dsgtemp.tsc3_)

b1dsgtopo_t b1dsgtopo = {
/*  Topological information
*/
    /* ground */ 1,
    /* nbod */ 5,
    /* ndof */ 5,
    /* ncons */ 5,
    /* nloop */ 1,
    /* nldof */ 1,
    /* nloopc */ 5,
    /* nball */ 0,
    /* nlball */ 0,
    /* npres */ 0,
    /* nuser */ 0,
    /* jtype[0] */ 1,
    /* jtype[1] */ 1,
    /* jtype[2] */ 1,
    /* jtype[3] */ 1,
    /* jtype[4] */ 1,
    /* jtype[5] */ 1,
    /* inb[0] */ -1,
    /* inb[1] */ 0,
    /* inb[2] */ 1,
    /* inb[3] */ 2,
    /* inb[4] */ 3,
    /* inb[5] */ -1,
    /* outb[0] */ 0,
    /* outb[1] */ 1,
    /* outb[2] */ 2,
    /* outb[3] */ 3,
    /* outb[4] */ 4,
    /* outb[5] */ 4,
    /* njntdof[0] */ 1,
    /* njntdof[1] */ 1,
    /* njntdof[2] */ 1,
    /* njntdof[3] */ 1,
    /* njntdof[4] */ 1,
    /* njntdof[5] */ 1,
    /* njntc[0] */ 0,
    /* njntc[1] */ 0,
    /* njntc[2] */ 0,
    /* njntc[3] */ 0,
    /* njntc[4] */ 0,
    /* njntc[5] */ 5,
    /* njntp[0] */ 0,
    /* njntp[1] */ 0,
    /* njntp[2] */ 0,
    /* njntp[3] */ 0,
    /* njntp[4] */ 0,
    /* njntp[5] */ 0,
    /* firstq[0] */ 0,
    /* firstq[1] */ 1,
    /* firstq[2] */ 2,
    /* firstq[3] */ 3,
    /* firstq[4] */ 4,
    /* firstq[5] */ 0,
    /* ballq[0] */ -104,
    /* ballq[1] */ -104,
    /* ballq[2] */ -104,
    /* ballq[3] */ -104,
    /* ballq[4] */ -104,
    /* ballq[5] */ -104,
    /* firstm[0] */ -1,
    /* firstm[1] */ -1,
    /* firstm[2] */ -1,
    /* firstm[3] */ -1,
    /* firstm[4] */ -1,
    /* firstm[5] */ 0,
    /* firstp[0] */ -1,
    /* firstp[1] */ -1,
    /* firstp[2] */ -1,
    /* firstp[3] */ -1,
    /* firstp[4] */ -1,
    /* firstp[5] */ -1,
    /* trans[0] */ 0,
    /* trans[1] */ 0,
    /* trans[2] */ 0,
    /* trans[3] */ 0,
    /* trans[4] */ 0,
    /* trans[5] */ 0,
};
b1dsginput_t b1dsginput = {
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

/* loop first inboard pin vectors */
    /* inbpin1[0][0] */ 0.,
    /* inbpin1[0][1] */ 0.,
    /* inbpin1[0][2] */ 1.,

/* loop second inboard pin vectors */
    /* inbpin2[0][0] */ 0.,
    /* inbpin2[0][1] */ 0.,
    /* inbpin2[0][2] */ 0.,

/* loop third inboard pin vectors */
    /* inbpin3[0][0] */ 0.,
    /* inbpin3[0][1] */ 0.,
    /* inbpin3[0][2] */ 0.,

/* loop inboard reference vectors */
    /* inbref[0][0] */ 0.,
    /* inbref[0][1] */ 0.,
    /* inbref[0][2] */ 0.,

/* loop body pin vectors */
    /* bodypin[0][0] */ 0.,
    /* bodypin[0][1] */ 0.,
    /* bodypin[0][2] */ 0.,

/* loop body reference vectors */
    /* bodyref[0][0] */ 0.,
    /* bodyref[0][1] */ 0.,
    /* bodyref[0][2] */ 0.,

/* loop bodytojoint vectors */
    /* lbtj[0][0] */ 0.,
    /* lbtj[0][1] */ -.1016,
    /* lbtj[0][2] */ 0.,

/* loop inbtojoint vectors */
    /* litj[0][0] */ 0.,
    /* litj[0][1] */ 0.,
    /* litj[0][2] */ 0.,

/* loop prescribed motion */
    /* lpres[0] */ 0.,

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
    /* inbpin1q[0][0] */ 0,
    /* inbpin1q[0][1] */ 0,
    /* inbpin1q[0][2] */ 0,
    /* inbpin2q[0][0] */ 0,
    /* inbpin2q[0][1] */ 0,
    /* inbpin2q[0][2] */ 0,
    /* inbpin3q[0][0] */ 0,
    /* inbpin3q[0][1] */ 0,
    /* inbpin3q[0][2] */ 0,
    /* inbrefq[0][0] */ 0,
    /* inbrefq[0][1] */ 0,
    /* inbrefq[0][2] */ 0,
    /* bodypinq[0][0] */ 0,
    /* bodypinq[0][1] */ 0,
    /* bodypinq[0][2] */ 0,
    /* bodyrefq[0][0] */ 0,
    /* bodyrefq[0][1] */ 0,
    /* bodyrefq[0][2] */ 0,
    /* lbtjq[0][0] */ 0,
    /* lbtjq[0][1] */ 3,
    /* lbtjq[0][2] */ 0,
    /* litjq[0][0] */ 1,
    /* litjq[0][1] */ 0,
    /* litjq[0][2] */ 0,
    /* lpresq[0] */ 0,

/* End of values from input file */

};
b1dsgstate_t b1dsgstate;
b1dsglhs_t b1dsglhs;
b1dsgrhs_t b1dsgrhs;
b1dsgtemp_t b1dsgtemp;


b1dsinit()
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
            b1dsseterr(7,25);
        }
    }
    for (k = 0; k < 5; k++) {
        if (mkq[k] == 1) {
            b1dsseterr(7,26);
        }
        for (i = 0; i < 3; i++) {
            if (rkq[k][i] == 1) {
                b1dsseterr(7,29);
            }
            if (riq[k][i] == 1) {
                b1dsseterr(7,30);
            }
            for (j = 0; j < 3; j++) {
                if (ikq[k][i][j] == 1) {
                    b1dsseterr(7,27);
                }
            }
        }
    }
    for (k = 0; k < 5; k++) {
        for (i = 0; i < 3; i++) {
            if (pinq[k][i] == 1) {
                b1dsseterr(7,28);
            }
        }
    }
    for (k = 0; k < 1; k++) {
        for (i = 0; i < 3; i++) {
            if (inbpin1q[k][i] == 1) {
                b1dsseterr(7,34);
            }
            if (inbpin2q[k][i] == 1) {
                b1dsseterr(7,34);
            }
            if (inbpin3q[k][i] == 1) {
                b1dsseterr(7,34);
            }
            if (inbrefq[k][i] == 1) {
                b1dsseterr(7,35);
            }
            if (bodypinq[k][i] == 1) {
                b1dsseterr(7,36);
            }
            if (bodyrefq[k][i] == 1) {
                b1dsseterr(7,37);
            }
            if (lbtjq[k][i] == 1) {
                b1dsseterr(7,38);
            }
            if (litjq[k][i] == 1) {
                b1dsseterr(7,39);
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
    b1dsserialno(&i);
    if (i != 70405) {
        b1dsseterr(7,41);
    }
    roustate = 1;
}

b1dsstate(timein,qin,uin)
    double timein,qin[5],uin[5];
{
/*
Compute kinematic information and store it in sdgstate.

Generated 21-Sep-2004 10:34:48 by SD/FAST, Kane's formulation
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
    double e1,e2,e3,e4;

    if ((roustate != 1) && (roustate != 2) && (roustate != 3)) {
        b1dsseterr(8,22);
        return;
    }
    if (roustate == 1) {
        for (i = 0; i < 5; i++) {
            if (presq[i] == 1) {
                b1dsseterr(8,31);
            }
        }
        for (i = 0; i < 1; i++) {
            if (lpresq[i] == 1) {
                b1dsseterr(8,40);
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
Compute Cio (dircos btw loop jt connected bodies)
*/
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
/*
Compute loop joint q-related temps
*/
    vt7[0][0] = (cnk[4][0][1]*lbtj[0][1]);
    vt7[0][1] = (cnk[4][1][1]*lbtj[0][1]);
    vt10c[0][0] = (rnk[4][0]+vt7[0][0]);
    vt10c[0][1] = (rnk[4][1]+vt7[0][1]);
    vt11[0][0] = (litj[0][0]-vt10c[0][0]);
/*
Compute dircos inside loop joints
*/
/*
Decompose loop joint dircos into Euler angles or parameters
*/
    if ((cnk[4][1][0] > 1.)  ) {
        temp[0] = 1.;
    } else {
        if ((cnk[4][1][0] < -1.)  ) {
            temp[1] = -1.;
        } else {
            temp[1] = cnk[4][1][0];
        }
        temp[0] = temp[1];
    }
    tsc2 = asin(temp[0]);
    if ((cnk[4][0][0] >= 0.)  ) {
        temp[0] = tsc2;
    } else {
        temp[0] = (3.14159265358979-tsc2);
    }
    e1 = temp[0];
    if ((e1 > 3.14159265358979)  ) {
        temp[0] = (e1-6.28318530717959);
    } else {
        temp[0] = e1;
    }
    eul1[0] = temp[0];
    ceul1[0] = cos(eul1[0]);
    seul1[0] = sin(eul1[0]);
/*
Compute constraint position errors
*/
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
Compute loop joint u-related temps
*/
    vt8[0][0] = -(lbtj[0][1]*wk[4][2]);
    vt12[0][0] = -(vnk[4][0]+(cnk[4][0][0]*vt8[0][0]));
    vt12[0][1] = -(vnk[4][1]+(cnk[4][1][0]*vt8[0][0]));
/*
Compute angvel inside loop joints
*/
/*
Compute derivatives of loop joint pseudo-coordinates
*/
/*
Compute qdot (kinematical equations)
*/
    qdot[0] = u[0];
    qdot[1] = u[1];
    qdot[2] = u[2];
    qdot[3] = u[3];
    qdot[4] = u[4];
/*
Compute constraint velocity errors
*/
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
    for (i = 0; i < 1; i++) {
        ltau[i] = 0.;
    }
    ltauflg = 0;
    fs0flg = 0;
/*
Set lqs -- b1dspsstate may override some
*/
    lq[0] = eul1[0];
/*
 Used 0.10 seconds CPU time,
 16384 additional bytes of memory.
 Equations contain   90 adds/subtracts/negates
                    130 multiplies
                      2 divides
                    241 assignments
*/
}

b1dsqdot(oqdot)
    double oqdot[5];
{
/*
Return position coordinate derivatives for tree joints.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(63,23);
        return;
    }
    for (i = 0; i <= 4; i++) {
        oqdot[i] = qdot[i];
    }
}

b1dsu2qdot(uin,oqdot)
    double uin[5],oqdot[5];
{
/*
Convert velocities to qdots.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(64,23);
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

b1dspsstate(lqin)
    double lqin[1];
{

    if (roustate != 2) {
        b1dsseterr(9,23);
        return;
    }
}

b1dsdovpk()
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
 0 additional bytes of memory.
 Equations contain   36 adds/subtracts/negates
                     32 multiplies
                      0 divides
                     50 assignments
*/
}

b1dsdoltau()
{

/*
Compute effect of loop hinge torques
*/
    if (ltauflg == 0) {
/*
Compute forces due to loop taus
*/
        ltauflg = 1;
    }
/*
 Used 0.00 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain    0 adds/subtracts/negates
                      0 multiplies
                      0 divides
                      0 assignments
*/
}

b1dsdoiner()
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

b1dsdofs0()
{

/*
Compute effect of all applied loads
*/
    if (fs0flg == 0) {
        b1dsdoltau();
        b1dsdoiner();
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
        Tstar[4][2] = -(ltau[0]+utk[4][2]);
/*
Compute fs0 (RHS ignoring constraints)
*/
        b1dsdovpk();
        fs0[0] = (utau[0]-((Tstar[4][2]+((Fstar[4][0]*Vpk[0][4][0])+(Fstar[4][1]
          *Vpk[0][4][1])))+((((Fstar[3][0]*Vpk[0][3][0])+(Fstar[3][1]*
          Vpk[0][3][1]))-utk[3][2])+((((Fstar[0][0]*rk[0][1])-utk[0][2])+(((
          Fstar[1][0]*Vpk[0][1][0])+(Fstar[1][1]*Vpk[0][1][1]))-utk[1][2]))+(((
          Fstar[2][0]*Vpk[0][2][0])+(Fstar[2][1]*Vpk[0][2][1]))-utk[2][2])))));
        fs0[1] = (utau[1]-((Tstar[4][2]+((Fstar[4][0]*Vpk[1][4][0])+(Fstar[4][1]
          *Vpk[1][4][1])))+((((Fstar[1][0]*rk[1][1])-utk[1][2])+(((Fstar[2][0]*
          Vpk[1][2][0])+(Fstar[2][1]*Vpk[1][2][1]))-utk[2][2]))+(((Fstar[3][0]*
          Vpk[1][3][0])+(Fstar[3][1]*Vpk[1][3][1]))-utk[3][2]))));
        fs0[2] = (utau[2]-((Tstar[4][2]+((Fstar[4][0]*Vpk[2][4][0])+(Fstar[4][1]
          *Vpk[2][4][1])))+(((Fstar[2][0]*rk[2][1])-utk[2][2])+(((Fstar[3][0]*
          Vpk[2][3][0])+(Fstar[3][1]*Vpk[2][3][1]))-utk[3][2]))));
        fs0[3] = (utau[3]-((Tstar[4][2]+((Fstar[4][0]*Vpk[3][4][0])+(Fstar[4][1]
          *Vpk[3][4][1])))+((Fstar[3][0]*rk[3][1])-utk[3][2])));
        fs0[4] = (utau[4]-(Tstar[4][2]+(Fstar[4][0]*rk[4][1])));
        fs0flg = 1;
    }
/*
 Used 0.02 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain   61 adds/subtracts/negates
                     37 multiplies
                      0 divides
                     16 assignments
*/
}

b1dsdomm(routine)
    int routine;
{
    int i;

    if (mmflg == 0) {
/*
Compute mass matrix (MM)
*/
        b1dsdovpk();
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
                b1dsseterr(routine,47);
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

b1dsdommldu(routine)
    int routine;
{
    int i;

    if (mmlduflg == 0) {
        b1dsdomm(routine);
/*
Numerically decompose the mass matrix
*/
        b1dsldudcomp(5,5,mmap,1e-13,workss,works,mm,mlo,mdi);
/*
Check for singular mass matrix
*/
        for (i = 0; i < 5; i++) {
            if (mdi[i] <= 1e-13) {
                b1dsseterr(routine,47);
            }
        }
        mmlduflg = 1;
    }
}

b1dslhs(routine)
    int routine;
{
/* Compute all remaining state- and force-dependent quantities
*/

    roustate = 2;
    b1dsdommldu(routine);
    b1dsdofs0();
}

b1dsmassmat(mmat)
    double mmat[5][5];
{
/* Return the system mass matrix (LHS)
*/
    int i,j;

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(57,23);
        return;
    }
    b1dsdomm(57);
    for (i = 0; i < 5; i++) {
        for (j = i; j <= 4; j++) {
            mmat[i][j] = mm[i][j];
            mmat[j][i] = mm[i][j];
        }
    }
}

b1dsfrcmat(fmat)
    double fmat[5];
{
/* Return the system force matrix (RHS), excluding constraints
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(58,23);
        return;
    }
    b1dsdofs0();
    for (i = 0; i < 5; i++) {
        fmat[i] = fs0[i];
    }
}

b1dsmfrc(imult)
    double imult[5];
{
/*
Calculate forces due to constraint multipliers.

*/

    lfci[0][0] = imult[2];
    lfci[0][1] = imult[3];
    lfci[0][2] = imult[4];
    lfc[0][0] = -((cnk[4][0][0]*lfci[0][0])+(cnk[4][1][0]*lfci[0][1]));
    lfc[0][1] = -((cnk[4][0][1]*lfci[0][0])+(cnk[4][1][1]*lfci[0][1]));
    lfc[0][2] = -lfci[0][2];
    ltci[0][0] = imult[0];
    ltci[0][1] = imult[1];
    ltc[0][0] = -((cnk[4][0][0]*ltci[0][0])+(cnk[4][1][0]*ltci[0][1]));
    ltc[0][1] = -((cnk[4][0][1]*ltci[0][0])+(cnk[4][1][1]*ltci[0][1]));
    Tinb[0][0] = (ltci[0][0]+(lfci[0][2]*vt10c[0][1]));
    Tinb[0][1] = (ltci[0][1]-(lfci[0][2]*vt10c[0][0]));
    Tinb[0][2] = ((lfci[0][1]*vt10c[0][0])-(lfci[0][0]*vt10c[0][1]));
    Toutb[0][0] = (ltc[0][0]+(lbtj[0][1]*lfc[0][2]));
    Toutb[0][2] = -(lbtj[0][1]*lfc[0][0]);
    mfk[4][0] = lfc[0][0];
    mfk[4][1] = lfc[0][1];
    mfk[4][2] = lfc[0][2];
    mtk[4][0] = Toutb[0][0];
    mtk[4][1] = ltc[0][1];
    mtk[4][2] = Toutb[0][2];
    Toutb[0][1] = ltc[0][1];
/*
 Used 0.01 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   14 adds/subtracts/negates
                     14 multiplies
                      0 divides
                     22 assignments
*/
}

b1dsequivht(tau)
    double tau[5];
{
/* Compute tree hinge torques to match effect of applied loads
*/
    double fstareq[5][3],tstareq[5][3];

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(56,23);
        return;
    }
/*
Compute forces due to loop-joint hinge torques
*/
    b1dsdoltau();
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
    tstareq[4][2] = -(ltau[0]+utk[4][2]);
/*
Compute taus (RHS ignoring constraints and inertial forces)
*/
    b1dsdovpk();
    tau[0] = (utau[0]-((tstareq[4][2]+((fstareq[4][0]*Vpk[0][4][0])+(
      fstareq[4][1]*Vpk[0][4][1])))+((((fstareq[3][0]*Vpk[0][3][0])+(
      fstareq[3][1]*Vpk[0][3][1]))-utk[3][2])+((((fstareq[0][0]*rk[0][1])-
      utk[0][2])+(((fstareq[1][0]*Vpk[0][1][0])+(fstareq[1][1]*Vpk[0][1][1]))-
      utk[1][2]))+(((fstareq[2][0]*Vpk[0][2][0])+(fstareq[2][1]*Vpk[0][2][1]))-
      utk[2][2])))));
    tau[1] = (utau[1]-((tstareq[4][2]+((fstareq[4][0]*Vpk[1][4][0])+(
      fstareq[4][1]*Vpk[1][4][1])))+((((fstareq[1][0]*rk[1][1])-utk[1][2])+(((
      fstareq[2][0]*Vpk[1][2][0])+(fstareq[2][1]*Vpk[1][2][1]))-utk[2][2]))+(((
      fstareq[3][0]*Vpk[1][3][0])+(fstareq[3][1]*Vpk[1][3][1]))-utk[3][2]))));
    tau[2] = (utau[2]-((tstareq[4][2]+((fstareq[4][0]*Vpk[2][4][0])+(
      fstareq[4][1]*Vpk[2][4][1])))+(((fstareq[2][0]*rk[2][1])-utk[2][2])+(((
      fstareq[3][0]*Vpk[2][3][0])+(fstareq[3][1]*Vpk[2][3][1]))-utk[3][2]))));
    tau[3] = (utau[3]-((tstareq[4][2]+((fstareq[4][0]*Vpk[3][4][0])+(
      fstareq[4][1]*Vpk[3][4][1])))+((fstareq[3][0]*rk[3][1])-utk[3][2])));
    tau[4] = (utau[4]-(tstareq[4][2]+(fstareq[4][0]*rk[4][1])));
/*
Op counts below do not include called subroutines
*/
/*
 Used 0.02 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain   60 adds/subtracts/negates
                     37 multiplies
                      0 divides
                     16 assignments
*/
}

b1dsfulltrq(udotin,multin,trqout)
    double udotin[5],multin[5],trqout[5];
{
/* Compute hinge torques which would produce indicated udots
*/
    double fstarr[5][3],tstarr[5][3],Otkr[5][3],Atir[5][3],Atkr[5][3];

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(61,23);
        return;
    }
/*
Compute forces due to loop-joint hinge torques
*/
    b1dsdoltau();
/*
Compute multiplier-generated forces
*/
    b1dsmfrc(multin);
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
    fstarr[4][0] = ((mfk[4][0]+ufk[4][0])+(mk[4]*(gk[4][0]-Atkr[4][0])));
    fstarr[4][1] = ((mfk[4][1]+ufk[4][1])+(mk[4]*(gk[4][1]-Atkr[4][1])));
    fstarr[4][2] = (mfk[4][2]+ufk[4][2]);
    tstarr[0][2] = (utk[0][2]-(ik[0][2][2]*udotin[0]));
    tstarr[1][2] = (utk[1][2]-(ik[1][2][2]*Otkr[1][2]));
    tstarr[2][2] = (utk[2][2]-(ik[2][2][2]*Otkr[2][2]));
    tstarr[3][2] = (utk[3][2]-(ik[3][2][2]*Otkr[3][2]));
    tstarr[4][0] = (mtk[4][0]+utk[4][0]);
    tstarr[4][1] = (mtk[4][1]+utk[4][1]);
    tstarr[4][2] = ((utk[4][2]+(ltau[0]+mtk[4][2]))-(ik[4][2][2]*Otkr[4][2]));
/*
Now calculate the torques
*/
    b1dsdovpk();
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
 8192 additional bytes of memory.
 Equations contain  105 adds/subtracts/negates
                     76 multiplies
                      0 divides
                     45 assignments
*/
}

b1dscomptrq(udotin,trqout)
    double udotin[5],trqout[5];
{
/* Compute hinge torques to produce these udots, ignoring constraints
*/
    int i;
    double multin[5];

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(60,23);
        return;
    }
    for (i = 0; i < 5; i++) {
        multin[i] = 0.;
    }
    b1dsfulltrq(udotin,multin,trqout);
}

b1dsmulttrq(multin,trqout)
    double multin[5],trqout[5];
{
/* Compute hinge trqs which would be produced by these mults.
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(65,23);
        return;
    }
    b1dsmfrc(multin);
    b1dsfsmult();
    for (i = 0; i < 5; i++) {
        trqout[i] = fs[i];
    }
}

b1dsfs0()
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

b1dsfsmult()
{

/*
Compute Fs (multiplier-generated forces only)
*/
    b1dsdovpk();
    fs[0] = (mtk[4][2]+((mfk[4][0]*Vpk[0][4][0])+(mfk[4][1]*Vpk[0][4][1])));
    fs[1] = (mtk[4][2]+((mfk[4][0]*Vpk[1][4][0])+(mfk[4][1]*Vpk[1][4][1])));
    fs[2] = (mtk[4][2]+((mfk[4][0]*Vpk[2][4][0])+(mfk[4][1]*Vpk[2][4][1])));
    fs[3] = (mtk[4][2]+((mfk[4][0]*Vpk[3][4][0])+(mfk[4][1]*Vpk[3][4][1])));
    fs[4] = (mtk[4][2]+(mfk[4][0]*rk[4][1]));
/*
 Used 0.01 seconds CPU time,
 0 additional bytes of memory.
 Equations contain    9 adds/subtracts/negates
                      9 multiplies
                      0 divides
                      5 assignments
*/
}

b1dsfsfull()
{

/*
Compute Fs (including all forces)
*/
    b1dsfsmult();
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

b1dsfsgenmult()
{

/*
Compute Fs (generic multiplier-generated forces)
*/
    b1dsdovpk();
    fs[0] = (mtau[0]+((mtk[4][2]+((mfk[4][0]*Vpk[0][4][0])+(mfk[4][1]*
      Vpk[0][4][1])))+((mtk[3][2]+((mfk[3][0]*Vpk[0][3][0])+(mfk[3][1]*
      Vpk[0][3][1])))+((mtk[2][2]+((mfk[2][0]*Vpk[0][2][0])+(mfk[2][1]*
      Vpk[0][2][1])))+((mtk[0][2]+(mfk[0][0]*rk[0][1]))+(mtk[1][2]+((mfk[1][0]*
      Vpk[0][1][0])+(mfk[1][1]*Vpk[0][1][1]))))))));
    fs[1] = (mtau[1]+((mtk[4][2]+((mfk[4][0]*Vpk[1][4][0])+(mfk[4][1]*
      Vpk[1][4][1])))+((mtk[3][2]+((mfk[3][0]*Vpk[1][3][0])+(mfk[3][1]*
      Vpk[1][3][1])))+((mtk[1][2]+(mfk[1][0]*rk[1][1]))+(mtk[2][2]+((mfk[2][0]*
      Vpk[1][2][0])+(mfk[2][1]*Vpk[1][2][1])))))));
    fs[2] = (mtau[2]+((mtk[4][2]+((mfk[4][0]*Vpk[2][4][0])+(mfk[4][1]*
      Vpk[2][4][1])))+((mtk[2][2]+(mfk[2][0]*rk[2][1]))+(mtk[3][2]+((mfk[3][0]*
      Vpk[2][3][0])+(mfk[3][1]*Vpk[2][3][1]))))));
    fs[3] = (mtau[3]+((mtk[3][2]+(mfk[3][0]*rk[3][1]))+(mtk[4][2]+((mfk[4][0]*
      Vpk[3][4][0])+(mfk[4][1]*Vpk[3][4][1])))));
    fs[4] = (mtau[4]+(mtk[4][2]+(mfk[4][0]*rk[4][1])));
/*
 Used 0.01 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   40 adds/subtracts/negates
                     25 multiplies
                      0 divides
                      5 assignments
*/
}

b1dsfsgenfull()
{

/*
Compute Fs (incl generic mult & other forces)
*/
    b1dsfsgenmult();
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

b1dsrhs()
{
/*
Generated 21-Sep-2004 10:34:48 by SD/FAST, Kane's formulation
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
/*
Compute hinge torques for loop joints
*/
    ltauc[0] = ltau[0];
    b1dsdoiner();
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
/*
Compute loop joint temps
*/
    vt18[0][0] = -(lbtj[0][1]*Onkb[4][2]);
    vt18[0][1] = -(lbtj[0][1]*w2w2[4]);
    vt19[0][0] = -(ank[4][0]+((cnk[4][0][0]*vt18[0][0])+(cnk[4][0][1]*vt18[0][1]
      )));
    vt19[0][1] = -(ank[4][1]+((cnk[4][1][0]*vt18[0][0])+(cnk[4][1][1]*vt18[0][1]
      )));
/*
Compute loop joint accelerations
*/
/*
Compute constraint acceleration errors
*/
    roustate = 3;
/*
 Used 0.04 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   60 adds/subtracts/negates
                     49 multiplies
                      0 divides
                    120 assignments
*/
}

b1dspseudo(lqout,luout)
    double lqout[1],luout[1];
{
/*
Return pseudo-coordinates for loop joints.

*/

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(18,23);
        return;
    }
    lqout[0] = eul1[0];
    luout[0] = wk[4][2];
}

b1dspsqdot(lqdout)
    double lqdout[1];
{
/*
Return pseudo-coordinate derivatives for loop joints.

*/

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(28,23);
        return;
    }
    lqdout[0] = wk[4][2];
}

b1dspsudot(ludout)
    double ludout[1];
{
/*
Return pseudo-coordinate accelerations for loop joints.

*/

    if (roustate != 3) {
        b1dsseterr(29,24);
        return;
    }
    ludout[0] = Onkb[4][2];
}

b1dsperr(errs)
    double errs[5];
{
/*
Return position constraint errors.

*/

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(26,23);
        return;
    }
    errs[0] = 0.;
    errs[1] = 0.;
    errs[2] = vt11[0][0];
    errs[3] = -vt10c[0][1];
    errs[4] = 0.;
}

b1dsverr(errs)
    double errs[5];
{
/*
Return velocity constraint errors.

*/

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(27,23);
        return;
    }
    errs[0] = 0.;
    errs[1] = 0.;
    errs[2] = vt12[0][0];
    errs[3] = vt12[0][1];
    errs[4] = 0.;
}

b1dsaerr(errs)
    double errs[5];
{
/*
Return acceleration constraint errors.

*/

    if (roustate != 3) {
        b1dsseterr(35,24);
        return;
    }
    errs[0] = 0.;
    errs[1] = 0.;
    errs[2] = vt19[0][0];
    errs[3] = vt19[0][1];
    errs[4] = 0.;
}
int 
b1dsindx(joint,axis)
    int joint,axis;
{
    int offs,gotit;

    if (b1dschkjaxis(36,joint,axis) != 0) {
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

b1dspresacc(joint,axis,prval)
    int joint,axis;
    double prval;
{

}

b1dspresvel(joint,axis,prval)
    int joint,axis;
    double prval;
{

}

b1dsprespos(joint,axis,prval)
    int joint,axis;
    double prval;
{

}

b1dsgetht(joint,axis,torque)
    int joint,axis;
    double *torque;
{

    if (b1dschkjaxis(30,joint,axis) != 0) {
        return;
    }
    if (roustate != 3) {
        b1dsseterr(30,24);
        return;
    }
    if (joint  >  4) {
        *torque = ltauc[b1dsindx(joint,axis)];
    } else {
        *torque = tauc[b1dsindx(joint,axis)];
    }
}

b1dshinget(joint,axis,torque)
    int joint,axis;
    double torque;
{

    if (b1dschkjaxis(10,joint,axis) != 0) {
        return;
    }
    if (roustate != 2) {
        b1dsseterr(10,23);
        return;
    }
    if (mfrcflg != 0) {
        if (joint > 4) {
            mltau[b1dsindx(joint,axis)] = mltau[b1dsindx(joint,axis)]+torque;
        } else {
            mtau[b1dsindx(joint,axis)] = mtau[b1dsindx(joint,axis)]+torque;
        }
    } else {
        fs0flg = 0;
        if (joint > 4) {
            ltau[b1dsindx(joint,axis)] = ltau[b1dsindx(joint,axis)]+torque;
            ltauflg = 0;
        } else {
            utau[b1dsindx(joint,axis)] = utau[b1dsindx(joint,axis)]+torque;
        }
    }
}

b1dspointf(body,point,force)
    int body;
    double point[3],force[3];
{
    double torque[3];

    if (b1dschkbnum(11,body) != 0) {
        return;
    }
    if (roustate != 2) {
        b1dsseterr(11,23);
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

b1dsbodyt(body,torque)
    int body;
    double torque[3];
{

    if (b1dschkbnum(12,body) != 0) {
        return;
    }
    if (roustate != 2) {
        b1dsseterr(12,23);
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

b1dsdoww(routine)
    int routine;
{
    double pp[5][5],dpp[5][5];
    int i,j,c;
    double sum;
    double dfk[5][3],dtk[5][3],dtau[5],dltci[1][3],dltc[1][3],dlfci[1][3],dlfc[1
      ][3];
    double dTinb[1][3],dToutb[1][3],dltaufi[1][3],dltaufo[1][3],dltauti[1][3],
      dltauto[1][3];
    double dfs[5],row[5],dinvrow[5];

    roustate = 2;
    if (wwflg == 0) {
/*
Compute constraint effects
*/
        b1dsdovpk();
        b1dsdommldu(routine);
/*
Constraint 0 (loop constraint)
*/
        dltc[0][0] = -cnk[4][0][0];
        dltc[0][1] = -cnk[4][0][1];
/* ... redundant */
/*
Constraint 1 (loop constraint)
*/
        dltc[0][0] = -cnk[4][1][0];
        dltc[0][1] = -cnk[4][1][1];
/* ... redundant */
/*
Constraint 2 (loop constraint)
*/
        dlfc[0][0] = -cnk[4][0][0];
        dlfc[0][1] = -cnk[4][0][1];
        dToutb[0][2] = -(dlfc[0][0]*lbtj[0][1]);
        dfs[0] = (dToutb[0][2]+((dlfc[0][0]*Vpk[0][4][0])+(dlfc[0][1]*
          Vpk[0][4][1])));
        dfs[1] = (dToutb[0][2]+((dlfc[0][0]*Vpk[1][4][0])+(dlfc[0][1]*
          Vpk[1][4][1])));
        dfs[2] = (dToutb[0][2]+((dlfc[0][0]*Vpk[2][4][0])+(dlfc[0][1]*
          Vpk[2][4][1])));
        dfs[3] = (dToutb[0][2]+((dlfc[0][0]*Vpk[3][4][0])+(dlfc[0][1]*
          Vpk[3][4][1])));
        dfs[4] = (dToutb[0][2]+(dlfc[0][0]*rk[4][1]));
        b1dsldubsl(5,5,mmap,mlo,dfs,row);
        b1dsldubsd(5,5,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 4; i++) {
            pp[2][i] = row[i];
            dpp[i][2] = dinvrow[i];
        }
        wmap[0] = 2;
/*
Constraint 3 (loop constraint)
*/
        dlfc[0][0] = -cnk[4][1][0];
        dlfc[0][1] = -cnk[4][1][1];
        dToutb[0][2] = -(dlfc[0][0]*lbtj[0][1]);
        dfs[0] = (dToutb[0][2]+((dlfc[0][0]*Vpk[0][4][0])+(dlfc[0][1]*
          Vpk[0][4][1])));
        dfs[1] = (dToutb[0][2]+((dlfc[0][0]*Vpk[1][4][0])+(dlfc[0][1]*
          Vpk[1][4][1])));
        dfs[2] = (dToutb[0][2]+((dlfc[0][0]*Vpk[2][4][0])+(dlfc[0][1]*
          Vpk[2][4][1])));
        dfs[3] = (dToutb[0][2]+((dlfc[0][0]*Vpk[3][4][0])+(dlfc[0][1]*
          Vpk[3][4][1])));
        dfs[4] = (dToutb[0][2]+(dlfc[0][0]*rk[4][1]));
        b1dsldubsl(5,5,mmap,mlo,dfs,row);
        b1dsldubsd(5,5,mmap,mdi,row,dinvrow);
        for (i = 0; i <= 4; i++) {
            pp[3][i] = row[i];
            dpp[i][3] = dinvrow[i];
        }
        wmap[1] = 3;
/*
Constraint 4 (loop constraint)
*/
/* ... redundant */
/*
Produce constraint coefficient matrix WW
*/
        for (c = 0; c <= 1; c++) {
            for (i = c; i <= 1; i++) {
                sum = 0.;
                for (j = 0; j <= 4; j++) {
                    sum = sum+pp[wmap[c]][j]*dpp[j][wmap[i]];
                }
                ww[wmap[c]][wmap[i]] = sum;
                ww[wmap[i]][wmap[c]] = sum;
            }
        }
/*
Form QR decomposition of WW
*/
        b1dsqrdcomp(5,5,2,2,wmap,wmap,ww,qraux,jpvt);
        wwflg = 1;
    }
/*
 Used 0.05 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   43 adds/subtracts/negates
                     35 multiplies
                      0 divides
                     64 assignments
*/
}

b1dsxudot0(routine,oudot0)
    int routine;
    double oudot0[5];
{
/*
Compute unconstrained equations
*/
    int i;

    b1dslhs(routine);
/*
Solve equations ignoring constraints
*/
    b1dsfs0();
    b1dsldubslv(5,5,mmap,works,mlo,mdi,fs,udot);
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

b1dsudot0(oudot0)
    double oudot0[5];
{

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(66,23);
        return;
    }
    b1dsxudot0(66,oudot0);
}

b1dssetudot(iudot)
    double iudot[5];
{
/*
Assign udots and advance to stage Dynamics Ready
*/
    int i;

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(68,23);
        return;
    }
    for (i = 0; i <= 4; i++) {
        udot[i] = iudot[i];
    }
    b1dsrhs();
}

b1dsxudotm(routine,imult,oudotm)
    int routine;
    double imult[5],oudotm[5];
{
/*
Compute udots due only to multipliers
*/
    int i;

    b1dslhs(routine);
    b1dsmfrc(imult);
    b1dsfsmult();
    b1dsldubslv(5,5,mmap,works,mlo,mdi,fs,udot);
    for (i = 0; i <= 4; i++) {
        oudotm[i] = udot[i];
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

b1dsudotm(imult,oudotm)
    double imult[5],oudotm[5];
{

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(67,23);
        return;
    }
    b1dsxudotm(67,imult,oudotm);
}

b1dsderiv(oqdot,oudot)
    double oqdot[5],oudot[5];
{
/*
This is the derivative section for a 5-body ground-based
system with 5 hinge degree(s) of freedom.
There are 5 constraints.
*/
    double workr[5],bb[5],b0[5],v0[5],p0[5];
    int iwork[5];
    int i,j;
    double udot0[5],udot1[5];

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(17,23);
        return;
    }
    if (stabvelq == 1) {
        b1dsseterr(17,32);
    }
    if (stabposq == 1) {
        b1dsseterr(17,33);
    }
    wsiz = 2;
/*
Compute unconstrained equations
*/
    b1dsxudot0(17,udot0);
    b1dsrhs();
    b1dsaerr(b0);
    if (stabvel  !=  0.) {
        b1dsverr(v0);
    }
    if (stabpos  !=  0.) {
        b1dsperr(p0);
    }
/*
Stabilize constraints using Baumgarte's method
*/
    for (i = 0; i <= 4; i++) {
        bb[i] = -b0[i];
    }
    if (stabvel  !=  0.) {
        for (i = 0; i <= 4; i++) {
            bb[i] = bb[i]-stabvel*v0[i];
        }
    }
    if (stabpos  !=  0.) {
        for (i = 0; i <= 4; i++) {
            bb[i] = bb[i]-stabpos*p0[i];
        }
    }
/*
Compute and decompose constraint matrix WW
*/
    b1dsdoww(17);
/*
Numerically solve for constraint multipliers
*/
    for (i = 0; i <= 4; i++) {
        mult[i] = 0.;
    }
    b1dsqrbslv(5,5,2,2,wmap,wmap,1e-13,workr,iwork,ww,qraux,jpvt,bb,mult,&wrank)
      ;
    for (i = 0; i <= 1; i++) {
        multmap[i] = 0;
    }
    for (i = 0; i < wrank; i++) {
        multmap[jpvt[i]] = 1;
    }
    j = 0;
    for (i = 0; i <= 1; i++) {
        if (multmap[i] != 0) {
            multmap[j] = wmap[i];
            j = j+1;
        }
    }
/*
Compute final udots
*/
    b1dsxudotm(17,mult,udot1);
    for (i = 0; i <= 4; i++) {
        udot[i] = udot0[i]+udot1[i];
    }
    b1dsrhs();
    for (i = 0; i <= 4; i++) {
        oqdot[i] = qdot[i];
    }
    for (i = 0; i <= 4; i++) {
        oudot[i] = udot[i];
    }
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   20 adds/subtracts/negates
                     10 multiplies
                      0 divides
                     35 assignments
*/
}
/*
Compute residuals for use with DAE integrator
*/

b1dsresid(eqdot,eudot,emults,resid)
    double eqdot[5],eudot[5],emults[5],resid[15];
{
    int i;
    double uderrs[5],p0[5];

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(16,23);
        return;
    }
    if (stabposq == 1) {
        b1dsseterr(16,33);
    }
    b1dsfulltrq(eudot,emults,uderrs);
    for (i = 0; i < 5; i++) {
        resid[i] = eqdot[i]-qdot[i];
    }
    for (i = 0; i < 5; i++) {
        resid[5+i] = uderrs[i];
    }
    b1dsverr(&resid[10]);
    if (stabpos  !=  0.) {
        b1dsperr(p0);
        for (i = 0; i < 5; i++) {
            resid[10+i] = resid[10+i]+stabpos*p0[i];
        }
    }
    for (i = 0; i < 5; i++) {
        udot[i] = eudot[i];
    }
    for (i = 0; i < 5; i++) {
        mult[i] = emults[i];
    }
    b1dsrhs();
/*
 Used 0.00 seconds CPU time,
 0 additional bytes of memory.
 Equations contain   10 adds/subtracts/negates
                      5 multiplies
                      0 divides
                     25 assignments
*/
}

b1dsmult(omults,owrank,omultmap)
    double omults[5];
    int *owrank,omultmap[5];
{
    int i;

    if (roustate != 3) {
        b1dsseterr(34,24);
        return;
    }
    for (i = 0; i < 5; i++) {
        omults[i] = mult[i];
        if (i <= wrank-1) {
            omultmap[i] = multmap[i];
        } else {
            omultmap[i] = -1;
        }
    }
    *owrank = wrank;
}

b1dsreac(force,torque)
    double force[6][3],torque[6][3];
{
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

    if (roustate != 3) {
        b1dsseterr(31,24);
        return;
    }
/*
Compute reaction forces for non-weld tree joints
*/
    ffkb[4][0] = (mfk[4][0]+ufk[4][0]);
    ffkb[4][1] = (mfk[4][1]+ufk[4][1]);
    ffkb[4][2] = (mfk[4][2]+ufk[4][2]);
    ttkb[4][0] = (mtk[4][0]+utk[4][0]);
    ttkb[4][1] = (mtk[4][1]+utk[4][1]);
    ttkb[4][2] = (utk[4][2]+(ltau[0]+mtk[4][2]));
    fc[4][0] = ((mk[4]*(AnkAtk[4][0]-gk[4][0]))-ffkb[4][0]);
    fc[4][1] = ((mk[4]*(AnkAtk[4][1]-gk[4][1]))-ffkb[4][1]);
    fc[4][2] = -ffkb[4][2];
    tc[4][0] = -(ttkb[4][0]+(fc[4][2]*rk[4][1]));
    tc[4][1] = -ttkb[4][1];
    tc[4][2] = ((ik[4][2][2]*Onkb[4][2])-(ttkb[4][2]-(fc[4][0]*rk[4][1])));
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
Compute reaction forces for loop joints
*/
    force[5][0] = lfc[0][0];
    torque[5][0] = ltc[0][0];
    force[5][1] = lfc[0][1];
    torque[5][1] = ltc[0][1];
    force[5][2] = lfc[0][2];
    torque[5][2] = ltau[0];
/*
Compute reaction forces for tree weld joints
*/
/*
 Used 0.04 seconds CPU time,
 8192 additional bytes of memory.
 Equations contain  105 adds/subtracts/negates
                     67 multiplies
                      0 divides
                    108 assignments
*/
}

b1dsmom(lm,am,ke)
    double lm[3],am[3],*ke;
{
/*
Compute system linear and angular momentum, and kinetic energy.

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
    double lk[5][3],hnk[5][3];

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(19,23);
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

b1dssys(mtoto,cm,icm)
    double *mtoto,cm[3],icm[3][3];
{
/*
Compute system total mass, and instantaneous center of mass and
inertia matrix.

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
    double ikcnkt[5][3][3];

    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(20,23);
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

b1dspos(body,pt,loc)
    int body;
    double pt[3],loc[3];
{
/*
Return inertial frame location of a point on a body.

*/
    double pv[3];

    if (b1dschkbnum(21,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(21,23);
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

b1dsvel(body,pt,velo)
    int body;
    double pt[3],velo[3];
{
/*
Return inertial frame velocity of a point on a body.

*/
    double pv[3];

    if (b1dschkbnum(22,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(22,23);
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

b1dsorient(body,dircos)
    int body;
    double dircos[3][3];
{
/*
Return orientation of body w.r.t. ground frame.

*/

    if (b1dschkbnum(23,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(23,23);
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

b1dsangvel(body,avel)
    int body;
    double avel[3];
{
/*
Return angular velocity of the body.

*/

    if (b1dschkbnum(24,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(24,23);
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

b1dstrans(frbod,ivec,tobod,ovec)
    int frbod;
    double ivec[3];
    int tobod;
    double ovec[3];
{
/*
Transform ivec from frbod frame to tobod frame.

*/
    double pv[3];

    if (b1dschkbnum(25,frbod) != 0) {
        return;
    }
    if (b1dschkbnum(25,tobod) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(25,23);
        return;
    }
    if (frbod == tobod) {
        b1dsvcopy(ivec,ovec);
        return;
    }
    if (frbod == -1) {
        b1dsvcopy(ivec,pv);
        ovec[0] = pv[0]*cnb[tobod][0][0]+pv[1]*cnb[tobod][1][0]+pv[2]*cnb[tobod
          ][2][0];
        ovec[1] = pv[0]*cnb[tobod][0][1]+pv[1]*cnb[tobod][1][1]+pv[2]*cnb[tobod
          ][2][1];
        ovec[2] = pv[0]*cnb[tobod][0][2]+pv[1]*cnb[tobod][1][2]+pv[2]*cnb[tobod
          ][2][2];
        return;
    }
    if (tobod == -1) {
        b1dsvcopy(ivec,pv);
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

b1dsrel2cart(coord,body,point,linchg,rotchg)
    int coord,body;
    double point[3],linchg[3],rotchg[3];
{
/* Return derivative of pt loc and body orient w.r.t. hinge rate
*/
    int x,i,gnd;
    double lin[3],pv[3];

    if ((coord < 0) || (coord > 4)) {
        b1dsseterr(59,45);
        return;
    }
    if (b1dschkbnum(59,body) != 0) {
        return;
    }
    if ((roustate != 2) && (roustate != 3)) {
        b1dsseterr(59,23);
        return;
    }
    gnd = -1;
    if (body == gnd) {
        x = -1;
    } else {
        x = firstq[body]+njntdof[body]-1;
    }
    if (x < coord) {
        b1dsvset(0.,0.,0.,linchg);
        b1dsvset(0.,0.,0.,rotchg);
        return;
    }
    b1dsdovpk();
    for (i = 0; i < 3; i++) {
        rotchg[i] = Wpk[coord][x][i];
        lin[i] = Vpk[coord][x][i];
    }
    if (body == gnd) {
        b1dsvcopy(point,pv);
    } else {
        pv[0] = rcom[body][0]+point[0];
        pv[1] = rcom[body][1]+point[1];
        pv[2] = rcom[body][2]+point[2];
    }
    b1dsvcross(rotchg,pv,linchg);
    b1dsvadd(linchg,lin,linchg);
}

b1dsacc(body,pt,accel)
    int body;
    double pt[3],accel[3];
{
/*
Return linear acceleration a point of the specified body.

*/
    double pv[3];

    if (b1dschkbnum(32,body) != 0) {
        return;
    }
    if (roustate != 3) {
        b1dsseterr(32,24);
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

b1dsangacc(body,aacc)
    int body;
    double aacc[3];
{
/*
Return angular acceleration of the body.

*/

    if (b1dschkbnum(33,body) != 0) {
        return;
    }
    if (roustate != 3) {
        b1dsseterr(33,24);
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

b1dsgrav(gravin)
    double gravin[3];
{

    b1dsseterr(1,19);
    roustate = 0;
}

b1dsmass(body,massin)
    int body;
    double massin;
{

    if (b1dschkbnum(2,body) != 0) {
        return;
    }
    if (body == -1) {
        b1dsseterr(2,15);
        return;
    }
    if (mkq[body] != 0) {
        mk[body] = massin;
        mkq[body] = 3;
    } else {
        b1dsseterr(2,19);
    }
    roustate = 0;
}

b1dsiner(body,inerin)
    int body;
    double inerin[3][3];
{
    int anyques;

    if (b1dschkbnum(3,body) != 0) {
        return;
    }
    if (body == -1) {
        b1dsseterr(3,15);
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
        b1dsseterr(3,19);
    }
    roustate = 0;
}

b1dsbtj(joint,btjin)
    int joint;
    double btjin[3];
{
    int anyques;

    if (b1dschkjnum(4,joint) != 0) {
        return;
    }
    anyques = 0;
    if (joint  >  4) {
        if (lbtjq[joint-5][0]  !=  0) {
            lbtj[joint-5][0] = btjin[0];
            lbtjq[joint-5][0] = 3;
            anyques = 1;
        }
        if (lbtjq[joint-5][1]  !=  0) {
            lbtj[joint-5][1] = btjin[1];
            lbtjq[joint-5][1] = 3;
            anyques = 1;
        }
        if (lbtjq[joint-5][2]  !=  0) {
            lbtj[joint-5][2] = btjin[2];
            lbtjq[joint-5][2] = 3;
            anyques = 1;
        }
    } else {
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
    }
    if (anyques == 0) {
        b1dsseterr(4,19);
    }
    roustate = 0;
}

b1dsitj(joint,itjin)
    int joint;
    double itjin[3];
{
    int anyques;

    if (b1dschkjnum(5,joint) != 0) {
        return;
    }
    anyques = 0;
    if (joint  >  4) {
        if (litjq[joint-5][0]  !=  0) {
            litj[joint-5][0] = itjin[0];
            litjq[joint-5][0] = 3;
            anyques = 1;
        }
        if (litjq[joint-5][1]  !=  0) {
            litj[joint-5][1] = itjin[1];
            litjq[joint-5][1] = 3;
            anyques = 1;
        }
        if (litjq[joint-5][2]  !=  0) {
            litj[joint-5][2] = itjin[2];
            litjq[joint-5][2] = 3;
            anyques = 1;
        }
    } else {
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
    }
    if (anyques == 0) {
        b1dsseterr(5,19);
    }
    roustate = 0;
}

b1dspin(joint,pinno,pinin)
    int joint,pinno;
    double pinin[3];
{
    int ljnum,anyques,offs;

    if (b1dschkjpin(6,joint,pinno) != 0) {
        return;
    }
    anyques = 0;
    if (joint  >  4) {
        ljnum = joint-5;
        if (pinno  ==  0) {
            if (inbpin1q[ljnum][0]  !=  0) {
                inbpin1[ljnum][0] = pinin[0];
                inbpin1q[ljnum][0] = 3;
                anyques = 1;
            }
            if (inbpin1q[ljnum][1]  !=  0) {
                inbpin1[ljnum][1] = pinin[1];
                inbpin1q[ljnum][1] = 3;
                anyques = 1;
            }
            if (inbpin1q[ljnum][2]  !=  0) {
                inbpin1[ljnum][2] = pinin[2];
                inbpin1q[ljnum][2] = 3;
                anyques = 1;
            }
        }
        if (pinno  ==  1) {
            if (inbpin2q[ljnum][0]  !=  0) {
                inbpin2[ljnum][0] = pinin[0];
                inbpin2q[ljnum][0] = 3;
                anyques = 1;
            }
            if (inbpin2q[ljnum][1]  !=  0) {
                inbpin2[ljnum][1] = pinin[1];
                inbpin2q[ljnum][1] = 3;
                anyques = 1;
            }
            if (inbpin2q[ljnum][2]  !=  0) {
                inbpin2[ljnum][2] = pinin[2];
                inbpin2q[ljnum][2] = 3;
                anyques = 1;
            }
        }
        if (pinno  ==  2) {
            if (inbpin3q[ljnum][0]  !=  0) {
                inbpin3[ljnum][0] = pinin[0];
                inbpin3q[ljnum][0] = 3;
                anyques = 1;
            }
            if (inbpin3q[ljnum][1]  !=  0) {
                inbpin3[ljnum][1] = pinin[1];
                inbpin3q[ljnum][1] = 3;
                anyques = 1;
            }
            if (inbpin3q[ljnum][2]  !=  0) {
                inbpin3[ljnum][2] = pinin[2];
                inbpin3q[ljnum][2] = 3;
                anyques = 1;
            }
        }
        if (pinno  ==  3) {
            if (inbrefq[ljnum][0]  !=  0) {
                inbref[ljnum][0] = pinin[0];
                inbrefq[ljnum][0] = 3;
                anyques = 1;
            }
            if (inbrefq[ljnum][1]  !=  0) {
                inbref[ljnum][1] = pinin[1];
                inbrefq[ljnum][1] = 3;
                anyques = 1;
            }
            if (inbrefq[ljnum][2]  !=  0) {
                inbref[ljnum][2] = pinin[2];
                inbrefq[ljnum][2] = 3;
                anyques = 1;
            }
        }
        if (pinno  ==  4) {
            if (bodypinq[ljnum][0]  !=  0) {
                bodypin[ljnum][0] = pinin[0];
                bodypinq[ljnum][0] = 3;
                anyques = 1;
            }
            if (bodypinq[ljnum][1]  !=  0) {
                bodypin[ljnum][1] = pinin[1];
                bodypinq[ljnum][1] = 3;
                anyques = 1;
            }
            if (bodypinq[ljnum][2]  !=  0) {
                bodypin[ljnum][2] = pinin[2];
                bodypinq[ljnum][2] = 3;
                anyques = 1;
            }
        }
        if (pinno  ==  5) {
            if (bodyrefq[ljnum][0]  !=  0) {
                bodyref[ljnum][0] = pinin[0];
                bodyrefq[ljnum][0] = 3;
                anyques = 1;
            }
            if (bodyrefq[ljnum][1]  !=  0) {
                bodyref[ljnum][1] = pinin[1];
                bodyrefq[ljnum][1] = 3;
                anyques = 1;
            }
            if (bodyrefq[ljnum][2]  !=  0) {
                bodyref[ljnum][2] = pinin[2];
                bodyrefq[ljnum][2] = 3;
                anyques = 1;
            }
        }
    } else {
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
    }
    if (anyques == 0) {
        b1dsseterr(6,19);
    }
    roustate = 0;
}

b1dspres(joint,axis,presin)
    int joint,axis,presin;
{
    int anyques;

    if (b1dschkjaxis(37,joint,axis) != 0) {
        return;
    }
    if ((presin != 0) && (presin != 1)) {
        b1dsseterr(37,20);
    }
    anyques = 0;
    if (joint  >  4) {
        if (lpresq[b1dsindx(joint,axis)]  !=  0) {
            if (presin  !=  0) {
                lpres[b1dsindx(joint,axis)] = 1.;
            } else {
                lpres[b1dsindx(joint,axis)] = 0.;
            }
            lpresq[b1dsindx(joint,axis)] = 3;
            anyques = 1;
        }
    } else {
        if (presq[b1dsindx(joint,axis)]  !=  0) {
            if (presin  !=  0) {
                pres[b1dsindx(joint,axis)] = 1.;
            } else {
                pres[b1dsindx(joint,axis)] = 0.;
            }
            presq[b1dsindx(joint,axis)] = 3;
            anyques = 1;
        }
    }
    if (anyques == 0) {
        b1dsseterr(37,19);
    }
    wwflg = 0;
}

b1dsstab(velin,posin)
    double velin,posin;
{

    stabvel = velin;
    stabvelq = 3;
    stabpos = posin;
    stabposq = 3;
}

b1dsgetgrav(gravout)
    double gravout[3];
{

    gravout[0] = grav[0];
    gravout[1] = grav[1];
    gravout[2] = grav[2];
}

b1dsgetmass(body,massout)
    int body;
    double *massout;
{

    if (b1dschkbnum(40,body) != 0) {
        return;
    }
    if (body == -1) {
        b1dsseterr(40,15);
        return;
    }
    *massout = mk[body];
}

b1dsgetiner(body,inerout)
    int body;
    double inerout[3][3];
{

    if (b1dschkbnum(41,body) != 0) {
        return;
    }
    if (body == -1) {
        b1dsseterr(41,15);
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

b1dsgetbtj(joint,btjout)
    int joint;
    double btjout[3];
{

    if (b1dschkjnum(42,joint) != 0) {
        return;
    }
    if (joint  >  4) {
        btjout[0] = lbtj[joint-5][0];
        btjout[1] = lbtj[joint-5][1];
        btjout[2] = lbtj[joint-5][2];
    } else {
        btjout[0] = rk[joint][0];
        btjout[1] = rk[joint][1];
        btjout[2] = rk[joint][2];
    }
}

b1dsgetitj(joint,itjout)
    int joint;
    double itjout[3];
{

    if (b1dschkjnum(43,joint) != 0) {
        return;
    }
    if (joint  >  4) {
        itjout[0] = litj[joint-5][0];
        itjout[1] = litj[joint-5][1];
        itjout[2] = litj[joint-5][2];
    } else {
        itjout[0] = ri[joint][0];
        itjout[1] = ri[joint][1];
        itjout[2] = ri[joint][2];
    }
}

b1dsgetpin(joint,pinno,pinout)
    int joint,pinno;
    double pinout[3];
{
    int ljnum,offs;

    if (b1dschkjpin(44,joint,pinno) != 0) {
        return;
    }
    if (joint  >  4) {
        ljnum = joint-5;
        if (pinno  ==  0) {
            pinout[0] = inbpin1[ljnum][0];
            pinout[1] = inbpin1[ljnum][1];
            pinout[2] = inbpin1[ljnum][2];
        }
        if (pinno  ==  1) {
            pinout[0] = inbpin2[ljnum][0];
            pinout[1] = inbpin2[ljnum][1];
            pinout[2] = inbpin2[ljnum][2];
        }
        if (pinno  ==  2) {
            pinout[0] = inbpin3[ljnum][0];
            pinout[1] = inbpin3[ljnum][1];
            pinout[2] = inbpin3[ljnum][2];
        }
        if (pinno  ==  3) {
            pinout[0] = inbref[ljnum][0];
            pinout[1] = inbref[ljnum][1];
            pinout[2] = inbref[ljnum][2];
        }
        if (pinno  ==  4) {
            pinout[0] = bodypin[ljnum][0];
            pinout[1] = bodypin[ljnum][1];
            pinout[2] = bodypin[ljnum][2];
        }
        if (pinno  ==  5) {
            pinout[0] = bodyref[ljnum][0];
            pinout[1] = bodyref[ljnum][1];
            pinout[2] = bodyref[ljnum][2];
        }
    } else {
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
}

b1dsgetpres(joint,axis,presout)
    int joint,axis,*presout;
{

    if (b1dschkjaxis(45,joint,axis) != 0) {
        return;
    }
    if (joint  >  4) {
        if (lpres[b1dsindx(joint,axis)]  !=  0.) {
            *presout = 1;
        } else {
            *presout = 0;
        }
    } else {
        if (pres[b1dsindx(joint,axis)]  !=  0.) {
            *presout = 1;
        } else {
            *presout = 0;
        }
    }
}

b1dsgetstab(velout,posout)
    double *velout,*posout;
{

    *velout = stabvel;
    *posout = stabpos;
}

b1dsinfo(info)
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
    info[11] = 2;
/* info entries from 12-49 are reserved */
}

b1dsjnt(joint,info,tran)
    int joint,info[50],tran[6];
{
    int i,offs;

    if (b1dschkjnum(48,joint) != 0) {
        return;
    }
    info[0] = jtype[joint];
    if (joint  <=  4) {
        info[1] = 0;
        offs = 0;
    } else {
        info[1] = 1;
        offs = 5;
    }
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

b1dscons(consno,info)
    int consno,info[50];
{

    if (b1dschkucnum(49,consno) != 0) {
        return;
    }
/* There are no user constraints in this problem. */
}
int 
b1dschkbnum(routine,bnum)
    int routine,bnum;
{

    if ((bnum < -1) || (bnum > 4)) {
        b1dsseterr(routine,15);
        return 1;
    }
    return 0;
}
int 
b1dschkjnum(routine,jnum)
    int routine,jnum;
{

    if ((jnum < 0) || (jnum > 5)) {
        b1dsseterr(routine,16);
        return 1;
    }
    return 0;
}
int 
b1dschkucnum(routine,ucnum)
    int routine,ucnum;
{

    if ((ucnum < 0) || (ucnum > -1)) {
        b1dsseterr(routine,21);
        return 1;
    }
    return 0;
}
int 
b1dschkjaxis(routine,jnum,axnum)
    int routine,jnum,axnum;
{
    int maxax;

    if (b1dschkjnum(routine,jnum) != 0) {
        return 1;
    }
    if ((axnum < 0) || (axnum > 6)) {
        b1dsseterr(routine,17);
        return 1;
    }
    maxax = njntdof[jnum]-1;
    if ((jtype[jnum] == 4) || (jtype[jnum] == 6) || (jtype[jnum] == 21)) {
        maxax = maxax+1;
    }
    if (axnum > maxax) {
        b1dsseterr(routine,18);
        return 1;
    }
    return 0;
}
int 
b1dschkjpin(routine,jnum,pinno)
    int routine,jnum,pinno;
{
    int maxax,pinok;

    if (b1dschkjnum(routine,jnum) != 0) {
        return 1;
    }
    if ((pinno < 0) || (pinno > 5)) {
        b1dsseterr(routine,17);
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
    if (jnum > 4) {
        if ((pinno == 3) && ((jtype[jnum] == 1) || (jtype[jnum] == 5) || (jtype[
          jnum] == 7) || (jtype[jnum] == 9))) {
            pinok = 1;
        }
        if (((pinno == 4) || (pinno == 5)) && ((jtype[jnum] != 4) && (jtype[jnum
          ] != 6))) {
            pinok = 1;
        }
    }
    if (pinok == 0) {
        b1dsseterr(routine,18);
        return 1;
    }
    return 0;
}

/* Convert state to form using 1-2-3 Euler angles for ball joints. */

b1dsst2ang(st,stang)
    double st[5],stang[5];
{
    int i;

    for (i = 0; i < 5; i++) {
        stang[i] = st[i];
    }
}

/* Convert 1-2-3 form of state back to Euler parameters for ball joints. */

b1dsang2st(stang,st)
    double stang[5],st[5];
{
    int i;

    for (i = 0; i < 5; i++) {
        st[i] = stang[i];
    }
}

/* Normalize Euler parameters in state. */

b1dsnrmsterr(st,normst,routine)
    double st[5],normst[5];
    int routine;
{
    int i;

    for (i = 0; i < 5; i++) {
        normst[i] = st[i];
    }
}

b1dsnormst(st,normst)
    double st[5],normst[5];
{

    b1dsnrmsterr(st,normst,0);
}

b1dsgentime(gentm)
    int *gentm;
{

    *gentm = 103448;
}
/*
Done. CPU seconds used: 0.59  Memory used: 966656 bytes.
Equation complexity:
  sdstate:    90 adds   130 multiplies     2 divides   241 assignments
  sdderiv:   503 adds   486 multiplies    14 divides   780 assignments
  sdresid:   225 adds   176 multiplies     0 divides   262 assignments
  sdreac:    105 adds    67 multiplies     0 divides   108 assignments
  sdmom:      38 adds    43 multiplies     0 divides    22 assignments
  sdsys:      57 adds    78 multiplies     0 divides    16 assignments
*/
