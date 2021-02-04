#define LOG_LEVEL CONFIG_DISPLAY_LOG_LEVEL
#include <logging/log.h>

#include "rcar_vspd.h"
#include "rcar_lifec.h"

#define DT_DRV_COMPAT rcar_vspd

LOG_MODULE_REGISTER(rcar_vspd);

struct rcar_vspd_data {
    int dummy;
};

#define MSTPB(reg, bit)	((reg) * 100 + (bit))

/* RPF */
#define VSPD_RPF_OFFSET			0x100

#define VSPD_RPF_SRC_BSIZE		0x0300
#define VSPD_RPF_SRC_ESIZE		0x0304
#define VSPD_RPF_INFMT			0x0308
#define VSPD_RPF_DSWAP			0x030c
#define VSPD_RPF_LOC			0x0310
#define VSPD_RPF_ALPH_SEL		0x0314
#define VSPD_RPF_VRTCOL_SET		0x0318
#define VSPD_RPF_MSK_CTRL		0x031c
#define VSPD_RPF_MSK_SET0		0x0320
#define VSPD_RPF_MSK_SET1		0x0324
#define VSPD_RPF_CKEY_CTRL		0x0328
#define VSPD_RPF_CKEY_SET0		0x032c
#define VSPD_RPF_CKEY_SET1		0x0330
#define VSPD_RPF_SRCM_PSTRIDE		0x0334
#define VSPD_RPF_SRCM_ASTRIDE		0x0338
#define VSPD_RPF_SRCM_ADDR_Y		0x033c
#define VSPD_RPF_SRCM_ADDR_C0		0x0340
#define VSPD_RPF_SRCM_ADDR_C1		0x0344
#define VSPD_RPF_SRCM_ADDR_AI		0x0348
#define VSPD_RPF_MULT_ALPHA		0x036c

typedef struct RcarVspdDLHdrList {
	uint32_t	numBytes;
	uint32_t	pList;
} RcarVspdDLHdrList_t;

/* DisplayList Header section - Table 32.46 in h/w manual */
typedef struct __attribute__ ((aligned (16))) RcarVspdDLHdr {
	uint32_t	numLists;	/* num_list_minus1 */
	RcarVspdDLHdrList_t lists[8];
	uint32_t	nextHdr;
	uint32_t	flags;
} RcarVspdDLHdr_t;

/* DisplayList Body section */
typedef struct __attribute__ ((aligned (8))) RcarVspdDLBody {
	uint32_t	addr;
	uint32_t	data;
} RcarVspdDLBody_t;

typedef struct RcarVspd {
	uint32_t	base;
	uint16_t	mstpb;
	uint16_t	lifec;
	uint16_t	fcpv_mstpb;
	uint16_t	fcpv_lifec;
	uint32_t	rpf;
	uint32_t	wpf;
	uint32_t	lif;

	/* Use DL header & DL body. Ext sections can be added only if needed */
	RcarVspdDLHdr_t		dlHdr;
	/* NOTE: Not using Extended DL. See setupDisplayList() */
	/*RcarVspdDLHdrExt_T	dlHdrExt; */

	RcarVspdDLBody_t	dlBody;

	uint16_t src_w;
	uint16_t src_h;
	void *src_buf;
	uint16_t src_stride;
} RcarVspd_t;



RcarVspd_t xVspd[] = {
	[RCAR_VSPD0] = {	/* VSPDL on H3, M3-N */
		.base = 0xFEA20000U,
		.mstpb = MSTPB(6, 23),
		.lifec = LIFEC_SLAVE_VSPD0,
		.fcpv_mstpb = MSTPB(6, 3),		/* FCPVD0 */
		.fcpv_lifec = LIFEC_MASTER_FCPVD0,
		.rpf = 0,
		.wpf = 0,
		.lif = 0,
	},

	[RCAR_VSPD1] = {
		.base = 0xFEA28000U,
		.mstpb = MSTPB(6, 22),
		.lifec = LIFEC_SLAVE_VSPD1,
		.fcpv_mstpb = MSTPB(6, 2),		/* FCPVD1 */
		.fcpv_lifec = LIFEC_MASTER_FCPVD1,
		.rpf = 0,
		.wpf = 0,
		.lif = 0,
	},

	/* H3, M3-W only */
	[RCAR_VSPD2] = {
		.base = 0xFEA30000U,
		.mstpb = MSTPB(6, 21),
		.lifec = LIFEC_SLAVE_VSPD2,
		.fcpv_mstpb = MSTPB(6, 1),		/* FCPVD2 */
		.fcpv_lifec = LIFEC_MASTER_FCPVD2,
		.rpf = 0,
		.wpf = 0,
		.lif = 0,
	},
};

static void vspd_write(RcarVspd_t *ch, uint32_t offset, uint32_t val)
{
	sys_write32(val, ch->base + offset);
}

static void vspd_rpf_write(RcarVspd_t *ch, uint32_t offset, uint32_t val)
{
	sys_write32(val, ch->base + ch->rpf * VSPD_RPF_OFFSET + offset);
}

static uint32_t vspd_rpf_read(RcarVspd_t *ch, uint32_t offset)
{
	return sys_read32(ch->base + ch->rpf * VSPD_RPF_OFFSET + offset);
}

static int vspd_init(const struct device *dev) {
    LOG_DBG("Initializing rcar vspd driver");


#define CONFIG_RCAR_DISPLAY_EARLY_HOST_INIT

#ifdef CONFIG_RCAR_DISPLAY_EARLY_HOST_INIT
    int ix;
    for (ix = 0; ix <=  RCAR_VSPD2; ix++) {
        RcarVspd_t *rcarVspd = &xVspd[ix];
        rcarVspd->src_buf = (void*) vspd_rpf_read(rcarVspd, VSPD_RPF_SRCM_ADDR_Y);

		uint32_t bsize = vspd_rpf_read(rcarVspd, VSPD_RPF_SRC_BSIZE);
		rcarVspd->src_w = bsize >> 16;
		rcarVspd->src_h = bsize & 0xffff;

		char msg[256];
		sprintf(msg, "%s: w %d, h %d\n", __func__, rcarVspd->src_w, rcarVspd->src_h);

		ram_console_dirty(msg);
    }
    

#else
    /* TODO:
        hdmi init, du_init, lvds_init, vspd_init

	assert(ch <= RCAR_VSPD2);

	
    
    
    */   
#endif	

    return 0;
}


static void vsdp_start(const struct device *dev) {}
static void vspd_stop (const struct device *dev) {}

static void * vspd_get_framebuffer(const struct device *dev, vspd_channel_t channel) {
    RcarVspd_t * rcarVspd = &xVspd[channel];
    return rcarVspd->src_buf;
}

static void vspd_get_resolution(const struct device *dev, vspd_channel_t channel, uint32_t * width, uint32_t * height) {
    RcarVspd_t * rcarVspd = &xVspd[channel];
    *width = rcarVspd->src_w;
	*height = rcarVspd->src_h;
}


static const struct vspd_driver_api  vspd_api = {
    .start = vsdp_start,
    .stop = vspd_stop,
    .get_framebuffer = vspd_get_framebuffer,
	.get_resolution = vspd_get_resolution,
};

static struct rcar_vspd_data rcar_vspd_data;

DEVICE_AND_API_INIT(vspd, DT_PROP(DT_N_S_vspd, label), &vspd_init,
		    &rcar_vspd_data, NULL, APPLICATION,
		    CONFIG_APPLICATION_INIT_PRIORITY, &vspd_api);

