

struct 	op_t	// FM operator settings
{
	uint8_t	ar;
	uint8_t	d1r;
	uint8_t	d2r;
	uint8_t	rr;
	uint8_t	d1l;
	uint8_t	kls;
	uint8_t	ame_ebs_kvs;
	uint8_t	out;
	uint8_t	f;
	uint8_t	rs_det;
};				// 10 bytes

struct aop_t
{
	uint8_t	egshft_fix_fixrg;
	uint8_t	osw_fine;
};			// 2 bytes

struct voice_t
{
	op_t	op[4];
	uint8_t	sy_fbl_alg;
	uint8_t	lfs;
	uint8_t	lfd;
	uint8_t	pmd;
	uint8_t	amd;
	uint8_t	pms_ams_lfw;
	uint8_t	transpose;
	uint8_t	pbr;
	uint8_t	ch_mo_su_po_pm;
	uint8_t	port;
	uint8_t	fc_vol;
	uint8_t	mw_pitch;
	uint8_t mw_ampli;
	uint8_t	bc_pitch;
	uint8_t	bc_ampli;
	uint8_t	bc_p_bias;
	uint8_t	bc_e_bias;
	char 	voice_name[10];
	uint8_t pr1;
	uint8_t pr2;
	uint8_t pr3;
	uint8_t pl1;
	uint8_t pl2;
	uint8_t pl3;
	aop_t	aop[4];
	uint8_t	rev;
	uint8_t fc_pitch;
	uint8_t fc_ampli;
};		// 84 bytes