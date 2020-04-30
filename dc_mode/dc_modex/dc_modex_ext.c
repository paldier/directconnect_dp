/*
 * DirectConnect provides a common interface for the network devices to achieve the full or partial
   acceleration services from the underlying packet acceleration engine
 * Copyright (c) 2017, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

/* Includes */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/interrupt.h>
#if defined(CONFIG_X86_INTEL_LGM)
#include <linux/pp_api.h>
#endif

#include <net/switch_api/lantiq_gsw_api.h>
#if IS_ENABLED(CONFIG_PPA)
#include <net/ppa/ppa_api.h>
#endif /* #if IS_ENABLED(CONFIG_PPA) */

#include <directconnect_dp_api.h>
#include <directconnect_dp_dcmode_api.h>
#include <directconnect_dp_debug.h>

/* Defines */
#define DCMODE_EXT_DRV_MODULE_NAME "dc_mode-ext"
#define DCMODE_EXT_DRV_MODULE_VERSION "1.0"

#define DCMODE_EXT_BRIDGE_FLOW_LEARNING    1

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define SWITCH_DEV "/dev/switch_api/1"

#define DCMODE_EXT_MAX_PORT          20
#define DCMODE_EXT_MAX_SUBIF_PER_DEV 16
#define DCMODE_EXT_SUBIFID_OFFSET    8
#define DCMODE_EXT_SUBIFID_MASK      0xF
#define DCMODE_EXT_GET_SUBIFIDX(subif_id) \
            ((subif_id >> DCMODE_EXT_SUBIFID_OFFSET) & DCMODE_EXT_SUBIFID_MASK)
#define MULTIPORT_WORKAROUND_MAX_SUBIF_NUM 8
#define MULTIPORT_WORKAROUND_SUBIFID_MASK \
            (MULTIPORT_WORKAROUND_MAX_SUBIF_NUM << DCMODE_EXT_SUBIFID_OFFSET)

#if defined(CONFIG_PRX300_CQM)
#define DC_DP_DEFINE_LOCK(lock) DEFINE_MUTEX(lock)
#define DC_DP_LOCK    mutex_lock
#define DC_DP_UNLOCK  mutex_unlock
#else /* #if defined(CONFIG_PRX300_CQM) */
#define DC_DP_DEFINE_LOCK(lock) DEFINE_SPINLOCK(lock)
#define DC_DP_LOCK    spin_lock_bh
#define DC_DP_UNLOCK  spin_unlock_bh
#endif /* #else */

#define DCMODE_EXT_MAX_DEV_NUM      4
#define DCMODE_EXT_MAX_DEV_PORT_NUM (DCMODE_EXT_MAX_DEV_NUM << 1)
#define DCMODE_EXT_DEF_UMT_PERIOD   200 /* in micro second */

#define DC_DP_MAX_SOC_CLASS        16

struct dcmode_ext_subif_info {
    struct net_device  *netif;   /*! pointer to net_device*/
};

struct dcmode_ext_dev_shared_info
{
#define DCMODE_DEV_STATUS_FREE    0x0
#define DCMODE_DEV_STATUS_USED    0x1
    int32_t status;
    int32_t port_id;
    int32_t alt_port_id; /* Multiport reference port id */
    int32_t ref_count;
    int32_t cqm_pid;
    int32_t umt_id;
    int32_t umt_period;
    uint32_t dma_ctrlid;
    uint32_t dma_cid;
    uint32_t dma_ch;
    int32_t num_bufpools;
    struct dc_dp_buf_pool *buflist;
    int32_t num_subif;
    struct dcmode_ext_subif_info subif_info[DCMODE_EXT_MAX_SUBIF_PER_DEV];
};

struct dcmode_ext_dev_info
{
#define DCMODE_EXT_DEV_STATUS_FREE    0x0
#define DCMODE_EXT_DEV_STATUS_USED    0x1
    int32_t status;
    int32_t port_id;
    uint32_t alloc_flags;
    int32_t num_subif;

    /* shared info */
    struct dcmode_ext_dev_shared_info *shared_info;
};
static struct dcmode_ext_dev_shared_info g_dcmode_ext_dev_shinfo[DCMODE_EXT_MAX_DEV_NUM];
static struct dcmode_ext_dev_info g_dcmode_ext_dev[DCMODE_EXT_MAX_DEV_PORT_NUM];
static struct dcmode_ext_dev_info *g_dcmode_ext_dev_p[DCMODE_EXT_MAX_PORT] = {NULL} ;
static struct dp_cap g_dp_cap = {0};
DC_DP_DEFINE_LOCK(g_dcmode_ext_dev_lock);
static int32_t g_dcmode_ext_init_ok = 0;

/* Function prototypes */
/* Local */
static int32_t
dcmode_ext_rx_cb(struct net_device *rxif, struct net_device *txif,
                 struct sk_buff *skb, int32_t len);
static int32_t
dcmode_ext_get_netif_subifid_cb(struct net_device *netif,
                                struct sk_buff *skb, void *subif_data,
                                uint8_t dst_mac[MAX_ETH_ALEN],
                                dp_subif_t *subif, uint32_t flags);

#if !defined(CONFIG_X86_INTEL_LGM) && !defined(CONFIG_PRX300_CQM)
static inline int32_t
dcmode_ext_setup_pmac_port(int32_t port_id, int32_t dma_cid, int32_t ref_port_id, uint32_t dev_cap_req, uint32_t flags);
static inline int32_t
dcmode_ext_alloc_ring_buffers(uint32_t num_bufs_req, int32_t *num_bufpools, struct dc_dp_buf_pool **buflist);
static inline void
dcmode_ext_free_ring_buffers(int32_t num_bufpools, struct dc_dp_buf_pool **buflist);
static inline void
dcmode_ext_cleanup_ring_resources(struct dcmode_ext_dev_info *dev_info, int32_t port_id, struct dc_dp_res *res, uint32_t flags);
#endif /* #if !defined(CONFIG_X86_INTEL_LGM) && !defined(CONFIG_PRX300_CQM) */
static inline int32_t
dcmode_ext_setup_ring_resources(struct dcmode_ext_dev_info *dev_info, int32_t port_id,
                                struct dp_dev_data *dp_device, struct dc_dp_res *res, uint32_t flags);
static inline uint8_t
_dc_dp_get_class2devqos(uint8_t *class2prio, uint8_t *prio2devqos, uint8_t class);

#if 0
#if defined(CONFIG_DIRECTCONNECT_DP_DBG) && CONFIG_DIRECTCONNECT_DP_DBG
static void _dc_dp_dump_raw_data(char *buf, int len, char *prefix_str);
static void _dc_dp_dump_rx_pmac(struct pmac_rx_hdr *pmac);
#endif /* #if defined(CONFIG_DIRECTCONNECT_DP_DBG) && CONFIG_DIRECTCONNECT_DP_DBG */
#endif /* #if 0 */

/*
 * ========================================================================
 * Local Interface API
 * ========================================================================
 */

static inline bool is_multiport(uint32_t alloc_flags)
{
    return (0 != (alloc_flags & DC_DP_F_MULTI_PORT));
}

static inline bool is_multiport_main(uint32_t alloc_flags)
{
    return ( (0 != (alloc_flags & DC_DP_F_MULTI_PORT)) &&
             (0 == (alloc_flags & DC_DP_F_SHARED_RES)) );
}

static inline bool is_multiport_sub(uint32_t alloc_flags)
{
    return ( (0 != (alloc_flags & DC_DP_F_MULTI_PORT)) &&
             (0 != (alloc_flags & DC_DP_F_SHARED_RES)) );
}

static inline bool is_multiport_main_by_subif(struct dp_subif *subif_id)
{
    return (DCMODE_EXT_GET_SUBIFIDX(subif_id->subif) >= MULTIPORT_WORKAROUND_MAX_SUBIF_NUM);
}

static inline void
multiport_wa_forward_map_subifid(struct dcmode_ext_dev_info *dev_ctx, struct dp_subif *subif_id)
{
    /* Multiport 1st device? */
    if ( is_multiport_main(dev_ctx->alloc_flags) )
        subif_id->subif |= MULTIPORT_WORKAROUND_SUBIFID_MASK;
    /* Multiport sub-sequent device? */
    else
        subif_id->port_id = dev_ctx->shared_info->port_id;
}

static inline void
multiport_wa_reverse_map_subifid(struct dcmode_ext_dev_info *dev_ctx, struct dp_subif *subif_id)
{
    /* Multiport 1st device? */
    if ( is_multiport_main_by_subif(subif_id) )
        subif_id->subif &= ~MULTIPORT_WORKAROUND_SUBIFID_MASK;

    /* Multiport sub-sequent device? */
    else
        subif_id->port_id = dev_ctx->shared_info->alt_port_id;
}

static inline bool is_multiport_sub_by_subif(dp_subif_t *subif_id)
{
    return (DCMODE_EXT_GET_SUBIFIDX(subif_id->subif) < MULTIPORT_WORKAROUND_MAX_SUBIF_NUM);
}

#if defined(CONFIG_X86_INTEL_LGM)
static inline bool
is_required_auto_detect_buffer_return(struct dc_dp_dev *devspec, uint32_t alloc_flags)
{
    bool ret = false;

    if ( (devspec->dev_cap_req & DC_DP_F_DEV_REQ_AUTO_DETECT_BUFFER_RETURN) ) {
        ret = true;

    /* Default to be set for FAST_WLAN */
    } else if ( (alloc_flags & DC_DP_F_FAST_WLAN) ||
                !(alloc_flags & DC_DP_F_FAST_DSL) ) {
        ret = true;
    }

    return ret;
}

static inline bool
is_required_buffer_marking(struct dc_dp_dev *devspec, uint32_t alloc_flags)
{
    bool ret = false;

    if ( (devspec->dev_cap_req & DC_DP_F_DEV_REQ_BUFFER_MARKING) ) {
        ret = true;

    /* Default to be set for FAST_WLAN */
    } else if ( (alloc_flags & DC_DP_F_FAST_WLAN) ||
                !(alloc_flags & DC_DP_F_FAST_DSL) ) {
        ret = true;
    }

    return ret;
}
#endif /* #if defined(CONFIG_X86_INTEL_LGM) */

static struct dcmode_ext_dev_shared_info *
dcmode_ext_alloc_dev_shared_ctx(int32_t port_id, int32_t ref_port_id, uint32_t alloc_flags)
{
    int32_t dev_shinfo_idx;
    struct dcmode_ext_dev_shared_info *dev_shinfo_ctx = NULL;
    struct dcmode_ext_dev_info *ref_dev_ctx = NULL;

    /* Multiport 2nd device? */
    if ( is_multiport_sub(alloc_flags) &&
         (NULL != (ref_dev_ctx = g_dcmode_ext_dev_p[ref_port_id])) ) {
        dev_shinfo_ctx = ref_dev_ctx->shared_info;
        if (NULL == dev_shinfo_ctx)
            goto err_out;

        dev_shinfo_ctx->alt_port_id = port_id;
        dev_shinfo_ctx->ref_count++;
    } else {
        /* Find a free device shinfo index */
        for (dev_shinfo_idx = 0; dev_shinfo_idx < DCMODE_EXT_MAX_DEV_NUM; dev_shinfo_idx++) {
            if (g_dcmode_ext_dev_shinfo[dev_shinfo_idx].status != DCMODE_EXT_DEV_STATUS_USED) {
                break;
            }
        }

        if (dev_shinfo_idx >= DCMODE_EXT_MAX_DEV_NUM) {
            DC_DP_ERROR("failed to allocate port as it reaches maximum directconnect device limit - %d!!!\n", DCMODE_EXT_MAX_DEV_NUM);
            goto err_out;
        }
        dev_shinfo_ctx = &g_dcmode_ext_dev_shinfo[dev_shinfo_idx];

        memset(dev_shinfo_ctx, 0, sizeof(struct dcmode_ext_dev_shared_info));
        dev_shinfo_ctx->status = DCMODE_EXT_DEV_STATUS_USED;

        /* Multiport 2nd radio? */
        if ( is_multiport_sub(alloc_flags) ) {
            dev_shinfo_ctx->port_id = ref_port_id;
            dev_shinfo_ctx->alt_port_id = port_id;
        } else
            dev_shinfo_ctx->port_id = port_id;

        dev_shinfo_ctx->ref_count = 1;
    }

err_out:
    return dev_shinfo_ctx;
}

static inline void
dcmode_ext_free_dev_shared_ctx(struct dcmode_ext_dev_shared_info *dev_shinfo_ctx)
{
    if (NULL != dev_shinfo_ctx) {
        dev_shinfo_ctx->ref_count--;
        if (0 == dev_shinfo_ctx->ref_count)
            memset(dev_shinfo_ctx, 0, sizeof(struct dcmode_ext_dev_shared_info));
    }
}

static struct dcmode_ext_dev_info *
dcmode_ext_alloc_dev_ctx(int32_t port_id, int32_t ref_port_id, uint32_t alloc_flags)
{
    int32_t dev_idx;
    struct dcmode_ext_dev_info *dev_ctx = NULL;
    struct dcmode_ext_dev_shared_info *dev_shinfo_ctx = NULL;

    /* Find a free device index */
    for (dev_idx = 0; dev_idx < DCMODE_EXT_MAX_DEV_PORT_NUM; dev_idx++) {
        if (g_dcmode_ext_dev[dev_idx].status != DCMODE_EXT_DEV_STATUS_USED) {
            break;
        }
    }

    if (dev_idx >= DCMODE_EXT_MAX_DEV_PORT_NUM) {
        DC_DP_ERROR("failed to allocate port as it reaches maximum directconnect device limit - %d!!!\n", DCMODE_EXT_MAX_DEV_PORT_NUM);
        goto out;
    }

    /* Allocate device shared context */
    dev_shinfo_ctx = dcmode_ext_alloc_dev_shared_ctx(port_id, ref_port_id, alloc_flags);
    if (NULL == dev_shinfo_ctx)
        goto out;

    dev_ctx = &g_dcmode_ext_dev[dev_idx];

    /* Reset DC ModeX device structure */
    memset(dev_ctx, 0, sizeof(struct dcmode_ext_dev_info));
    dev_ctx->status = DCMODE_EXT_DEV_STATUS_USED;
    dev_ctx->port_id = port_id;
    dev_ctx->alloc_flags = alloc_flags;
    dev_ctx->shared_info = dev_shinfo_ctx;

    g_dcmode_ext_dev_p[port_id] = dev_ctx;

out:
    return dev_ctx;
}

static inline void
dcmode_ext_free_dev_ctx(struct dcmode_ext_dev_info *dev_ctx)
{
    if (NULL != dev_ctx) {
        /* Free device shared context */
        dcmode_ext_free_dev_shared_ctx(dev_ctx->shared_info);

        g_dcmode_ext_dev_p[dev_ctx->port_id] = NULL;
        memset(dev_ctx, 0, sizeof(struct dcmode_ext_dev_info));
    }
}

#if !defined(CONFIG_X86_INTEL_LGM) && !defined(CONFIG_PRX300_CQM)
static inline int32_t
dcmode_ext_setup_pmac_port(int32_t port_id, int32_t dma_cid, int32_t ref_port_id, uint32_t dev_cap_req, uint32_t flags)
{
    u8 i = 0, j = 0;
    GSW_API_HANDLE gswr;
    GSW_PMAC_Eg_Cfg_t egCfg;
    GSW_PMAC_Ig_Cfg_t igCfg;
    GSW_register_t regCfg;

    /* Do the GSW-R configuration */
    gswr = gsw_api_kopen(SWITCH_DEV);
    if (gswr == 0) {
        DC_DP_ERROR("Open SWAPI device FAILED!!!\n");
        return -EIO;
    }

    /* FIXME : setup FCS/PMAC setting based on device request */

    /* GSWIP-R PMAC Egress Configuration Table */
    DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "PMAC_EG_CFG_SET for GSW-R.\n");
    for (i = 0; i <= 15; i++) {
        for (j = 0; j <= 3; j++) {
            memset((void *)&egCfg, 0x00, sizeof(egCfg));
            egCfg.nRxDmaChanId  = 0;
            if ( (dev_cap_req & DC_DP_F_DEV_REQ_TX_PMAC) )
                egCfg.bPmacEna  = 1;
            else
                egCfg.bPmacEna  = 0;
            if ( (dev_cap_req & DC_DP_F_DEV_REQ_TX_FCS) )
                egCfg.bFcsEna   = 1;
            else
                egCfg.bFcsEna   = 0;
            egCfg.bRemL2Hdr     = 0;
            egCfg.numBytesRem   = 0;
            egCfg.nResDW1       = 0;
            egCfg.nRes1DW0      = 0;
            egCfg.nRes2DW0      = 0;
            egCfg.nDestPortId   = port_id;
            egCfg.nTrafficClass = i;
            egCfg.bMpe1Flag     = 0;
            egCfg.bMpe2Flag     = 0;
            egCfg.bEncFlag      = 0;
            egCfg.bDecFlag      = 0;
            egCfg.nFlowIDMsb    = j;
            egCfg.bTCEnable     = 1;

            gsw_api_kioctl(gswr, GSW_PMAC_EG_CFG_SET, (unsigned int)&egCfg);
        }
    }

    /* GSWIP-R PMAC Ingress Configuration Table */
    DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "PMAC_IG_CFG_SET for GSW-R.\n");
    memset((void *)&igCfg, 0x00, sizeof(igCfg));
    igCfg.nTxDmaChanId  = dma_cid;
    if ( (dev_cap_req & DC_DP_F_DEV_REQ_RX_PMAC) )
        igCfg.bPmacPresent  = 1;
    else
        igCfg.bPmacPresent  = 0;
    igCfg.bSpIdDefault  = 1;
    igCfg.eSubId        = GSW_PMAC_IG_CFG_SRC_DMA_DESC;
    igCfg.bClassDefault = 0;
    igCfg.bClassEna     = 0;
    igCfg.bErrPktsDisc  = 1;

    igCfg.bPmapDefault  = 1;
    igCfg.bPmapEna      = 0;

    igCfg.defPmacHdr[0] = 0;
    igCfg.defPmacHdr[1] = 0;
    igCfg.defPmacHdr[2] = (ref_port_id << 4);
    igCfg.defPmacHdr[3] = 0x80;
    igCfg.defPmacHdr[4] = 0;
    if ( (dev_cap_req & DC_DP_F_DEV_REQ_RX_FCS) )
        igCfg.defPmacHdr[4] |= 0x80;
    else
        igCfg.defPmacHdr[4] &= ~0x80;
    igCfg.defPmacHdr[5] = 0;
    igCfg.defPmacHdr[6] = 0xFF;
    igCfg.defPmacHdr[7] = 0xFF;

    gsw_api_kioctl(gswr, GSW_PMAC_IG_CFG_SET, (unsigned int)&igCfg);

    /* No Rx FCS, then allow any short packet without padding */
    /* SDMA_PRIO:USIGN */
    #define SDMA_PRIO_REG_BASE 0xBC1
    #define SDMA_PRIO_REG_PORT_STEP 6
    #define SDMA_PRIO_REG_USIGN 0x4

    memset((void *)&regCfg, 0x00, sizeof(regCfg));
    regCfg.nRegAddr = SDMA_PRIO_REG_BASE + (SDMA_PRIO_REG_PORT_STEP * port_id);
    gsw_api_kioctl(gswr, GSW_REGISTER_GET, (unsigned int)&regCfg);

    if ( (dev_cap_req & DC_DP_F_DEV_REQ_RX_FCS) )
        regCfg.nData &= ~SDMA_PRIO_REG_USIGN;
    else
        regCfg.nData |= SDMA_PRIO_REG_USIGN;

    gsw_api_kioctl(gswr, GSW_REGISTER_SET, (unsigned int)&regCfg);

    /* FIMXE : setup based on DSL or not */
        /* Allow traffic from one VAP to any VAP */

        /* PCE_PCTRL_3:IGPTRM */
        #define PCE_PCTRL_3_REG_BASE 0x483
        #define PCE_PCTRL_3_REG_PORT_STEP 10
        #define PCE_PCTRL_3_REG_IGPTRM 0x4000

        memset((void *)&regCfg, 0x00, sizeof(regCfg));
        regCfg.nRegAddr = PCE_PCTRL_3_REG_BASE + (PCE_PCTRL_3_REG_PORT_STEP * port_id);
        gsw_api_kioctl(gswr, GSW_REGISTER_GET, (unsigned int)&regCfg);
        regCfg.nData |= PCE_PCTRL_3_REG_IGPTRM;
        gsw_api_kioctl(gswr, GSW_REGISTER_SET, (unsigned int)&regCfg);

        /* PCE_IGPTRM:SUBx */
        #define PCE_IGPTRM_REG_BASE 0x544
        #define PCE_IGPTRM_REG_PORT_STEP 16
        #define PCE_IGPTRM_REG_SUB_ALL 0xFFFF

        memset((void *)&regCfg, 0x00, sizeof(regCfg));
        regCfg.nRegAddr = PCE_IGPTRM_REG_BASE + (PCE_IGPTRM_REG_PORT_STEP * port_id);
        gsw_api_kioctl(gswr, GSW_REGISTER_GET, (unsigned int)&regCfg);
        regCfg.nData |= PCE_IGPTRM_REG_SUB_ALL;
        gsw_api_kioctl(gswr, GSW_REGISTER_SET, (unsigned int)&regCfg);

    gsw_api_kclose(gswr);

    DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "GSW PMAC Init Done.\n");
    return 0;
}

static inline int32_t
dcmode_ext_alloc_ring_buffers(uint32_t num_bufs_req, int32_t *num_bufpools, struct dc_dp_buf_pool **buflist)
{
    int32_t i;
    uint32_t order;
    uint32_t max_buf_pool_num;
    uint32_t max_bufs_req_sz;
    struct dc_dp_buf_pool *buflist_base = NULL;
    size_t buflist_sz;
    size_t num_buflist_entries;
    uint32_t num_buf_req_rem;
    uint32_t tmp_num_bufs_req;
    uint32_t tmp_buf_pool_sz;
    uint8_t *buf_addr_base = NULL;

    if (num_bufs_req <= 0) {
        return -1;
    }

    if (DC_DP_PKT_BUF_SIZE_DEFAULT < PAGE_SIZE)
        max_buf_pool_num = PAGE_SIZE / DC_DP_PKT_BUF_SIZE_DEFAULT;
    else
        max_buf_pool_num = 1;

    max_bufs_req_sz = num_bufs_req * DC_DP_PKT_BUF_SIZE_DEFAULT;
    num_buflist_entries = (num_bufs_req + (max_buf_pool_num - 1)) / max_buf_pool_num;

    buflist_sz = (num_buflist_entries * sizeof(struct dc_dp_buf_pool)); /* virt buflist size */

    /* Allocate Tx buffers */
    DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "Allocating %d DMA1-Tx buffer lists.\n", (uint32_t)num_buflist_entries);
    buflist_base = (struct dc_dp_buf_pool *) kmalloc(buflist_sz, GFP_KERNEL);
    if (!buflist_base) {
        DC_DP_ERROR("failed to allocate %d buffer lists!!!\n", (uint32_t)num_buflist_entries);
        return -ENOMEM;
    }
    memset((void *)buflist_base, 0, buflist_sz);

    DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "Allocating %d DMA1-Tx buffer pools.\n", (uint32_t)num_buflist_entries);
    num_buf_req_rem = num_bufs_req;
    for (i = 0; i < num_buflist_entries; i++) {
        tmp_num_bufs_req = MIN(num_buf_req_rem, max_buf_pool_num);
        tmp_buf_pool_sz = tmp_num_bufs_req * DC_DP_PKT_BUF_SIZE_DEFAULT;
        order = get_order(tmp_buf_pool_sz);

        buf_addr_base = (uint8_t *)__get_free_pages(GFP_KERNEL, order);
        if (!buf_addr_base) {
            DC_DP_ERROR("failed to allocate pool %d of size %d KB!!!\n",
                            (i + 1), (tmp_buf_pool_sz >> 10));
            goto err_out_free_buf;
        }

        /* Buffer pool */
        buflist_base[i].pool = (void *)buf_addr_base;
        buflist_base[i].phys_pool = (void *)virt_to_phys(buf_addr_base);
        buflist_base[i].size = tmp_buf_pool_sz;

        num_buf_req_rem -= tmp_num_bufs_req;
    }

    /* Return */
    *num_bufpools = num_buflist_entries;
    *buflist = buflist_base;

    return 0;

err_out_free_buf:
    dcmode_ext_free_ring_buffers(num_buflist_entries, &buflist_base);

    return -ENOMEM;
}

static inline void
dcmode_ext_free_ring_buffers(int32_t num_bufpools, struct dc_dp_buf_pool **buflist_base)
{
    int32_t i;
    uint32_t order;
    struct dc_dp_buf_pool *buflist;

    if (NULL == buflist_base)
        return;

    buflist = *buflist_base;

    /* De-allocate Tx buffer pool */
    if (buflist) {
        DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "De-allocating %d DMA1-Tx buffer pools.\n", num_bufpools);
        for (i = 0; i < num_bufpools; i++) {
            if (buflist[i].pool && buflist[i].size) {
                order = get_order(buflist[i].size);
                free_pages((unsigned long)buflist[i].pool, order);
                buflist[i].pool = NULL;
                buflist[i].size = 0;
            }
        }

        DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "De-allocating %d buffer lists.\n", num_bufpools);
        kfree(buflist);
        *buflist_base = NULL;
    }
}

static inline void
dcmode_ext_cleanup_ring_resources(struct dcmode_ext_dev_info *dev_ctx, int32_t port_id, struct dc_dp_res *res, uint32_t flags)
{
    /* De-allocate Tx buffer pool */
    dcmode_ext_free_ring_buffers(dev_ctx->shared_info->num_bufpools, &dev_ctx->shared_info->buflist);
}
#endif /* #if !defined(CONFIG_X86_INTEL_LGM) && !defined(CONFIG_PRX300_CQM) */

static inline int32_t
dcmode_ext_setup_ring_resources(struct dcmode_ext_dev_info *dev_ctx, int32_t port_id,
                                struct dp_dev_data *dp_device, struct dc_dp_res *res, uint32_t flags)
{
    int32_t ret = DC_DP_SUCCESS;
#if !defined(CONFIG_X86_INTEL_LGM)
    int32_t buflist_idx;
#if defined(CONFIG_PRX300_CQM)
    uint32_t **v_buflist_base = NULL;
    uint32_t **p_buflist_base = NULL;
#else /* #if defined(CONFIG_PRX300_CQM) */
    int32_t num_bufpools = 0;
    struct dc_dp_buf_pool *buflist = NULL;
    int32_t i;
    int32_t j;
#endif /* #else */
#endif /* #if !defined(CONFIG_X86_INTEL_LGM) */

    /* rx_out ring */
    res->rings.dev2soc.phys_base = dp_device->rx_ring[0].out_enq_paddr;
    res->rings.dev2soc.base = phys_to_virt((phys_addr_t)dp_device->rx_ring[0].out_enq_paddr);
    res->rings.dev2soc.size = dp_device->rx_ring[0].out_enq_ring_size;
    res->rings.dev2soc.desc_dwsz = 4;
#if defined(CONFIG_X86_INTEL_LGM)
    res->rings.dev2soc.policy_base = dp_device->rx_ring[0].rx_policy_base;
    //res->rings.dev2soc.pool_id = dp_device->rx_ring[0].rx_poolid;
    //res->rings.dev2soc.high_4bits = dp_device->rx_ring[0].;
    res->rings.rxout_temp_dw3 = 0;
    /* Pool_Policy[23:16] */
    res->rings.rxout_temp_dw3 |= ((res->rings.dev2soc.policy_base & 0xFF) << 14);
    /* Source_Pool[27:24] */
    res->rings.rxout_temp_dw3 |= (res->rings.dev2soc.pool_id & 0xF);
#endif

    /* rx_in ring is being used? */
    if (dp_device->rx_ring[0].in_alloc_paddr) {
        res->rings.dev2soc_ret.phys_base = dp_device->rx_ring[0].in_alloc_paddr;
        res->rings.dev2soc_ret.base = phys_to_virt((phys_addr_t)dp_device->rx_ring[0].in_alloc_paddr);
        res->rings.dev2soc_ret.size = dp_device->rx_ring[0].in_alloc_ring_size;
#if defined(CONFIG_X86_INTEL_LGM)
        res->rings.dev2soc_ret.desc_dwsz = 2;
#else /* #if defined(CONFIG_X86_INTEL_LGM) */
        res->rings.dev2soc_ret.desc_dwsz = 4;
#endif /* #else */

    /* rx_in ring same as rx_out ring */
    } else {
        res->rings.dev2soc_ret = res->rings.dev2soc;
    }

    dev_ctx->shared_info->cqm_pid = dp_device->rx_ring[0].out_enq_port_id;
    dev_ctx->shared_info->dma_ch = dp_device->rx_ring[0].out_dma_ch_to_gswip;

    /* tx_in ring */
    res->rings.soc2dev.phys_base = dp_device->tx_ring[0].in_deq_paddr;
    res->rings.soc2dev.base = phys_to_virt((phys_addr_t)dp_device->tx_ring[0].in_deq_paddr);
    res->rings.soc2dev.size = dp_device->tx_ring[0].in_deq_ring_size;
    res->rings.soc2dev.desc_dwsz = 4;

    /* tx_out ring */
    res->rings.soc2dev_ret.phys_base = dp_device->tx_ring[0].out_free_paddr;
    res->rings.soc2dev_ret.base = phys_to_virt((phys_addr_t)dp_device->tx_ring[0].out_free_paddr);
    res->rings.soc2dev_ret.size = dp_device->tx_ring[0].out_free_ring_size;
#if defined(CONFIG_X86_INTEL_LGM) || defined(CONFIG_PRX300_CQM)
    res->rings.soc2dev_ret.desc_dwsz = 2;
#else /* #if defined(CONFIG_X86_INTEL_LGM) || defined(CONFIG_PRX300_CQM) */
    res->rings.soc2dev_ret.desc_dwsz = 1;
#endif /* #else */
    res->rings.soc2dev_ret.policy_base = dp_device->tx_ring[0].txout_policy_base;
    res->rings.soc2dev_ret.pool_id = dp_device->tx_ring[0].tx_poolid;
#if defined(CONFIG_X86_INTEL_LGM)
    //res->rings.soc2dev_ret.high_4bits = dp_device->tx_ring[0].;
    res->rings.txout_temp_dw3 = 0;
    /* Buffer Pointer[26:23] */
    res->rings.txout_temp_dw3 |= ((res->rings.soc2dev_ret.high_4bits & 0xF) << 23);
    /* Pool_Policy[21:14] */
    res->rings.txout_temp_dw3 |= ((res->rings.soc2dev_ret.policy_base & 0xFF) << 14);
    /* Source_Pool[3:0] */
    res->rings.txout_temp_dw3 |= (res->rings.soc2dev_ret.pool_id & 0xF);
#elif defined(CONFIG_PRX300_CQM) /* #if defined(CONFIG_X86_INTEL_LGM) */
    res->rings.txout_temp_dw3 = 0;
    /* Policy[22:20] */
    res->rings.txout_temp_dw3 |= ((res->rings.soc2dev_ret.policy_base & 0x7) << 20);
    /* Pool[18:16] */
    res->rings.txout_temp_dw3 |= ((res->rings.soc2dev_ret.pool_id & 0x7) << 16);
#endif /* #elif defined(CONFIG_PRX300_CQM) */

    /* FIXME : Tx buffer allocation */

    /* Allocate Rx buffers */
    res->num_bufpools = dp_device->rx_ring[0].num_pkt;
    dev_ctx->shared_info->num_bufpools = res->num_bufpools;
    /* NOTE : In LGM, no buflist resource update required */

#if defined(CONFIG_PRX300_CQM)
    res->buflist = (struct dc_dp_buf_pool *) kmalloc((res->num_bufpools * sizeof(struct dc_dp_buf_pool)), GFP_KERNEL);
    if (!res->buflist) {
        DC_DP_ERROR("failed to allocate %d buffer lists!!!\n", res->num_bufpools);
        ret = DC_DP_FAILURE;
        goto err_out;
    }

    v_buflist_base = (uint32_t **)dp_device->rx_ring[0].pkt_list_vaddr;
    p_buflist_base = (uint32_t **)dp_device->rx_ring[0].pkt_base_vaddr;
    for (buflist_idx = 0; buflist_idx < res->num_bufpools; buflist_idx++) {
        res->buflist[buflist_idx].pool = (void *)v_buflist_base[buflist_idx];
        res->buflist[buflist_idx].phys_pool = (void *)p_buflist_base[buflist_idx];
        res->buflist[buflist_idx].size = dp_device->rx_ring[0].rx_pkt_size;
    }
#elif !defined(CONFIG_X86_INTEL_LGM) /* #if defined(CONFIG_PRX300_CQM) */
    ret = dcmode_ext_alloc_ring_buffers(res->num_bufs_req, &num_bufpools, &buflist);
    if (ret) {
        DC_DP_ERROR("failed to register dev as tx buffer allocation failure!!!\n");
        goto err_out;
    }
    dev_ctx->shared_info->num_bufpools = num_bufpools;
    dev_ctx->shared_info->buflist = buflist;

    res->num_bufpools = res->num_bufs_req;
    res->buflist = (struct dc_dp_buf_pool *) kmalloc((res->num_bufpools * sizeof(struct dc_dp_buf_pool)), GFP_KERNEL);
    if (!res->buflist) {
        DC_DP_ERROR("failed to allocate %d buffer lists!!!\n", res->num_bufs_req);
        ret = DC_DP_FAILURE;
        goto err_out_free_buf;
    }

    buflist_idx = 0;
    for (i = 0; i < num_bufpools; i++) {
        for (j = 0; j < buflist[i].size; j += DC_DP_PKT_BUF_SIZE_DEFAULT) {
            res->buflist[buflist_idx].pool = buflist[i].pool + j;
            res->buflist[buflist_idx].phys_pool = buflist[i].phys_pool + j;
            res->buflist[buflist_idx].size = MIN(buflist[i].size, DC_DP_PKT_BUF_SIZE_DEFAULT);
            buflist_idx++;
        }
    }
#endif /* #else */

    return ret;

#if !defined(CONFIG_X86_INTEL_LGM)
#if !defined(CONFIG_PRX300_CQM)
err_out_free_buf:
    dcmode_ext_free_ring_buffers(num_bufpools, &buflist);
#endif /* #if !defined(CONFIG_PRX300_CQM) */

err_out:
    return ret;
#endif /* #if !defined(CONFIG_X86_INTEL_LGM) */
}

#if 0
#if defined(CONFIG_DIRECTCONNECT_DP_DBG) && CONFIG_DIRECTCONNECT_DP_DBG
static void
_dc_dp_dump_rx_pmac(struct pmac_rx_hdr *pmac)
{
    int i;
    unsigned char *p = (char *)pmac;

    if (!pmac) {
        pr_err("dump_rx_pmac pmac NULL ??\n");
        return ;
    }

    pr_info("PMAC at 0x%p: ", p);
    for (i = 0; i < 8; i++)
        pr_info("0x%02x ", p[i]);
    pr_info("\n");

    /*byte 0 */
    pr_info("  byte 0:res=%d ver_done=%d ip_offset=%d\n", pmac->res1,
           pmac->ver_done, pmac->ip_offset);
    /*byte 1 */
    pr_info("  byte 1:tcp_h_offset=%d tcp_type=%d\n", pmac->tcp_h_offset,
           pmac->tcp_type);
    /*byte 2 */
    pr_info("  byte 2:ppid=%d class=%d\n", pmac->sppid, pmac->class);
    /*byte 3 */
    pr_info("  byte 3:res=%d pkt_type=%d\n", pmac->res2, pmac->pkt_type);
    /*byte 4 */
    pr_info("  byte 4:res=%d redirect=%d res2=%d src_sub_inf_id=%d\n",
           pmac->res3, pmac->redirect, pmac->res4, pmac->src_sub_inf_id);
    /*byte 5 */
    pr_info("  byte 5:src_sub_inf_id2=%d\n", pmac->src_sub_inf_id2);
    /*byte 6 */
    pr_info("  byte 6:port_map=%d\n", pmac->port_map);
    /*byte 7 */
    pr_info("  byte 7:port_map2=%d\n", pmac->port_map2);
}
#endif /* #if defined(CONFIG_DIRECTCONNECT_DP_DBG) && CONFIG_DIRECTCONNECT_DP_DBG */
#endif /* #if 0 */

#define DC_DP_DEV_CLASS_MASK    0x7
static inline uint8_t
_dc_dp_get_class2devqos(uint8_t class2prio[], uint8_t prio2devqos[], uint8_t class)
{
    uint8_t devqos;
    uint8_t prio;

    class = (class & 0x0F);
    prio = class2prio[class];

    prio = (prio & DC_DP_DEV_CLASS_MASK);
    devqos = prio2devqos[prio];

    return devqos;
}

static inline void
get_soc_capability(uint32_t *cap)
{
    /* Linux offload capability */
    *cap = DC_DP_F_HOST_CAP_SG;
    if (g_dp_cap.tx_hw_chksum)
        *cap |= DC_DP_F_HOST_CAP_HW_CSUM;
    if (g_dp_cap.rx_hw_chksum)
        *cap |= DC_DP_F_HOST_CAP_RXCSUM;
    if (g_dp_cap.hw_tso) {
        *cap |= DC_DP_F_HOST_CAP_TSO;
        *cap |= DC_DP_F_HOST_CAP_TSO6;
    }

#if !defined(CONFIG_X86_INTEL_LGM)
#ifdef CONFIG_PPA_LRO
    *cap |= DC_DP_F_HOST_CAP_LRO;
#endif /* #ifdef CONFIG_PPA_LRO */

    /* FCS capability */
    *cap |= DC_DP_F_HOST_CAP_TX_FCS;
    *cap |= DC_DP_F_HOST_CAP_RX_FCS;
    *cap |= DC_DP_F_HOST_CAP_TX_WO_FCS;
    *cap |= DC_DP_F_HOST_CAP_RX_WO_FCS;

    /* PMAC capability */
    *cap |= DC_DP_F_HOST_CAP_TX_PMAC;
    *cap |= DC_DP_F_HOST_CAP_RX_PMAC;
    *cap |= DC_DP_F_HOST_CAP_RX_WO_PMAC;
#endif /* #if !defined(CONFIG_X86_INTEL_LGM) */

    /* QoS */
    *cap |= DC_DP_F_HOST_CAP_HW_QOS | DC_DP_F_HOST_CAP_HW_QOS_WAN;
    *cap |= DC_DP_F_HOST_CAP_DEVQOS;

#if defined(CONFIG_X86_INTEL_LGM)
    *cap |= DC_DP_F_HOST_CAP_AUTO_DETECT_BUFFER_RETURN;
    *cap |= DC_DP_F_HOST_CAP_BUFFER_MARKING;
#endif /* #if defined(CONFIG_X86_INTEL_LGM) */
}

#if defined(CONFIG_X86_INTEL_LGM)
static int32_t
dcmode_ext_get_subif_by_dev(struct net_device *dev, struct dp_subif *subif)
{
    int32_t ret = DC_DP_FAILURE;
    int32_t dev_idx;
    int32_t subif_idx;

    DC_DP_LOCK(&g_dcmode_ext_dev_lock);
    for (dev_idx = 0; dev_idx < DCMODE_EXT_MAX_DEV_NUM; dev_idx++) {
        if (g_dcmode_ext_dev_shinfo[dev_idx].status == DCMODE_DEV_STATUS_FREE)
            continue;

        for (subif_idx = 0; subif_idx < DCMODE_EXT_MAX_SUBIF_PER_DEV; subif_idx++) {
            if (g_dcmode_ext_dev_shinfo[dev_idx].subif_info[subif_idx].netif == dev) {
                subif->port_id = g_dcmode_ext_dev_shinfo[dev_idx].port_id;
                subif->subif = (subif_idx << DCMODE_EXT_SUBIFID_OFFSET);

                ret = DC_DP_SUCCESS;
                goto out;
            }
        }
    }

out:
    DC_DP_UNLOCK(&g_dcmode_ext_dev_lock);
    return ret;
}
#endif /* #if defined(CONFIG_X86_INTEL_LGM) */

/*
 * ========================================================================
 * DirectConnect Driver Interface API (SoC specific)
 * ========================================================================
 */
static int32_t
dcmode_ext_get_host_capability(struct dc_dp_host_cap *cap, uint32_t flags)
{
    int32_t ret = DC_DP_FAILURE;

    if (cap) {
        cap->fastpath.support = 1;
#if defined(CONFIG_X86_INTEL_LGM)
        cap->fastpath.hw_dcmode = DC_DP_MODE_TYPE_1_EXT;
#elif defined(CONFIG_PRX300_CQM) /* #if defined(CONFIG_X86_INTEL_LGM) */
        cap->fastpath.hw_dcmode = DC_DP_MODE_TYPE_0_EXT;
#else /* #elif defined(CONFIG_PRX300_CQM) */
        cap->fastpath.hw_dcmode = DC_DP_MODE_TYPE_0;
#endif /* #else */

        if (g_dp_cap.umt.umt_hw_auto.enable == 1) {
            if (g_dp_cap.umt.umt_hw_auto.rx_accumulate == 1)
                cap->fastpath.hw_cmode.dev2soc_write = DC_DP_F_DCCNTR_MODE_CUMULATIVE;
            if (g_dp_cap.umt.umt_hw_auto.rx_incremental == 1)
                cap->fastpath.hw_cmode.dev2soc_write |= DC_DP_F_DCCNTR_MODE_INCREMENTAL;
            if (g_dp_cap.umt.umt_hw_auto.tx_accumulate == 1)
                cap->fastpath.hw_cmode.soc2dev_write |= DC_DP_F_DCCNTR_MODE_CUMULATIVE;
            if (g_dp_cap.umt.umt_hw_auto.tx_incremental == 1)
                cap->fastpath.hw_cmode.soc2dev_write |= DC_DP_F_DCCNTR_MODE_INCREMENTAL;
        }

        cap->fastpath.hw_cmode.soc2dev_write |= DC_DP_F_DCCNTR_MODE_LITTLE_ENDIAN;
        cap->fastpath.hw_cmode.dev2soc_write |= DC_DP_F_DCCNTR_MODE_LITTLE_ENDIAN;

        get_soc_capability(&cap->fastpath.hw_cap);
        ret = DC_DP_SUCCESS;
    }

    return ret;
}

static int32_t
dcmode_ext_register_dev_ex(void *ctx,
                           struct module *owner, uint32_t port_id,
                           struct net_device *dev, struct dc_dp_cb *datapathcb,
                           struct dc_dp_res *resources, struct dc_dp_dev *devspec,
                           int32_t ref_port_id, uint32_t alloc_flags, uint32_t flags)
{
    int32_t ret;
    struct dcmode_ext_dev_info *dev_ctx = (struct dcmode_ext_dev_info *)ctx;
    dp_cb_t dp_cb = {0};
    struct dp_dev_data dp_device = {0};
    int32_t umt_idx;

    DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "dev_ctx=%p, owner=%p, port_id=%u, dev=%p, datapathcb=%p, "
                    "resources=%p, dev_spec=%p, flags=0x%08X\n",
                    dev_ctx, owner, port_id, dev, datapathcb, resources, devspec, flags);

    /* De-register */
    if (flags & DC_DP_F_DEREGISTER) {

        DC_DP_LOCK(&g_dcmode_ext_dev_lock);

        if ( !((NULL != dev_ctx) && (NULL != dev_ctx->shared_info)) ) {
            ret = DC_DP_FAILURE;
            goto err_unlock_out;
        }

        /* De-register DC ModeX device from DC Common layer */
        dc_dp_register_dcmode_device(owner, port_id, dev, dev_ctx, DC_DP_DCMODE_DEV_DEREGISTER);

        /* De-register device from DP Lib */
        /* Skip, iff sub-sequent Multiport device? */
        if ( !is_multiport_sub(dev_ctx->alloc_flags) )
            dp_register_dev_ext(0, owner, port_id, &dp_cb, &dp_device, DP_F_DEREGISTER);

#if !defined(CONFIG_X86_INTEL_LGM) && !defined(CONFIG_PRX300_CQM)
        /* For the last device */
        if ( (1 == dev_ctx->shared_info->ref_count) ) {
            /* Cleanup ring resources */
            DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "De-configuring DMA1-Tx channel 0x%x.\n",
                        dev_ctx->shared_info->dma_ch);
            dcmode_ext_cleanup_ring_resources(dev_ctx, dev_ctx->shared_info->port_id, resources, flags);
        }
#endif /* #if !defined(CONFIG_X86_INTEL_LGM) && !defined(CONFIG_PRX300_CQM) */

        /* Free DC ModeX device context */
        dcmode_ext_free_dev_ctx(dev_ctx);

        DC_DP_UNLOCK(&g_dcmode_ext_dev_lock);

        DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "Success, returned %d.\n", DC_DP_SUCCESS);
        return DC_DP_SUCCESS;
    }

    /* Validate input arguments */
    if (resources->num_dccntr != 1 && !resources->dccntr) {
        DC_DP_ERROR("failed to register device for the port_id %d!!!\n", port_id);
        ret = DC_DP_FAILURE;
        goto err_out;
    }

    DC_DP_LOCK(&g_dcmode_ext_dev_lock);

    if (NULL != g_dcmode_ext_dev_p[port_id]) {
        ret = DC_DP_FAILURE;
        goto err_unlock_out;
    }

    dev_ctx = dcmode_ext_alloc_dev_ctx(port_id, ref_port_id, alloc_flags);
    if (NULL == dev_ctx) {
        ret = DC_DP_FAILURE;
        goto err_unlock_out;
    }

    /* Skip, iff sub-sequent Multiport device? */
    if ( !is_multiport_sub(alloc_flags) ) {

        /* Datapath Library callback registration */
        dp_cb.rx_fn = dcmode_ext_rx_cb;
        dp_cb.stop_fn = datapathcb->stop_fn;
        dp_cb.restart_fn = datapathcb->restart_fn;
        if ( is_multiport(alloc_flags) )
            dp_cb.get_subifid_fn = dcmode_ext_get_netif_subifid_cb;
        else
            dp_cb.get_subifid_fn = dc_dp_get_netif_subifid; // Exported from DC common layer
        dp_cb.reset_mib_fn = datapathcb->reset_mib_fn;
        dp_cb.get_mib_fn = datapathcb->get_mib_fn;

        /* Update DP device structure */
        dp_device.num_rx_ring = 1;
        dp_device.num_tx_ring = 1;
        dp_device.num_umt_port = resources->num_dccntr;

        /* Rx ring */
        dp_device.rx_ring[0].out_enq_ring_size = resources->rings.dev2soc.size;
        dp_device.rx_ring[0].num_pkt = resources->num_bufs_req;
        dp_device.rx_ring[0].rx_pkt_size = DC_DP_PKT_BUF_SIZE_DEFAULT;
#if defined(CONFIG_X86_INTEL_LGM)
        dp_device.rx_ring[0].prefill_pkt_num = 0;
#else /* #if defined(CONFIG_X86_INTEL_LGM) */
        dp_device.rx_ring[0].prefill_pkt_num = dp_device.rx_ring[0].num_pkt;
#endif /* #else */
        dp_device.rx_ring[0].out_msg_mode = UMT_RXOUT_MSG_SUB;
        dp_device.rx_ring[0].out_qos_mode = DP_RXOUT_BYPASS_QOS_ONLY;

        /* Tx ring */
        dp_device.tx_ring[0].in_deq_ring_size = resources->rings.soc2dev.size;
        dp_device.tx_ring[0].num_tx_pkt = resources->tx_num_bufs_req;
        dp_device.tx_ring[0].tx_pkt_size = DC_DP_PKT_BUF_SIZE_DEFAULT;
        //dp_device.tx_ring[0].gpid_info. = ;

#if defined(CONFIG_X86_INTEL_LGM)
        if (is_required_auto_detect_buffer_return(devspec, alloc_flags) ) {
            dp_device.tx_ring[0].f_out_auto_free = 1;
            devspec->dc_cap |= DC_DP_F_HOST_CAP_AUTO_DETECT_BUFFER_RETURN;
        }
#endif /* #if defined(CONFIG_X86_INTEL_LGM) */

        /* TODO : GPID info */

        /* UMT info */
        for (umt_idx = 0; umt_idx < dp_device.num_umt_port; umt_idx++) {
            dp_device.umt[umt_idx].ctl.enable = 1;
            if (resources->dccntr[umt_idx].dev2soc_dccntr_timer > 0)
                dp_device.umt[umt_idx].ctl.msg_interval = resources->dccntr[umt_idx].dev2soc_dccntr_timer;
            else
                dp_device.umt[umt_idx].ctl.msg_interval = DCMODE_EXT_DEF_UMT_PERIOD;

            pr_info("%s: umt_mode = %d \n", __func__, resources->dccntr[umt_idx].umt_mode);
            if ( resources->dccntr[umt_idx].umt_mode == DC_DP_UMT_MODE_HW_AUTO) {
                dp_device.umt[umt_idx].ctl.daddr = (dma_addr_t)resources->dccntr[umt_idx].dev2soc_ret_enq_phys_base;
                dp_device.umt[umt_idx].ctl.msg_mode = UMT_MSG_SELFCNT;
            } else if (resources->dccntr[umt_idx].umt_mode == DC_DP_UMT_MODE_SW) {
                dp_device.umt[umt_idx].ctl.daddr = (dma_addr_t)resources->dccntr[umt_idx].dev2soc_ret_enq_base;
                dp_device.umt[umt_idx].ctl.msg_mode = UMT_MSG_SELFCNT;
                //dp_device.umt[umt_idx].usr_msg = ;
            } else if (resources->dccntr[umt_idx].umt_mode == DC_DP_UMT_MODE_HW_USER) {
                dp_device.umt[umt_idx].ctl.daddr = (dma_addr_t)resources->dccntr[umt_idx].dev2soc_ret_enq_phys_base;
                dp_device.umt[umt_idx].ctl.msg_mode = UMT_MSG_USER_MODE;
            }

            if ( (resources->dccntr[umt_idx].soc_write_dccntr_mode & DC_DP_F_DCCNTR_MODE_CUMULATIVE) )
                dp_device.umt[umt_idx].ctl.cnt_mode = UMT_CNT_ACC;
            else
                dp_device.umt[umt_idx].ctl.cnt_mode = UMT_CNT_INC;
        }

#if defined(CONFIG_X86_INTEL_LGM)
        if (is_required_buffer_marking(devspec, alloc_flags)) {
            dp_device.enable_cqm_meta = 1;
            devspec->dc_cap |= DC_DP_F_HOST_CAP_BUFFER_MARKING;
        }
#endif /* #if defined(CONFIG_X86_INTEL_LGM) */
        dp_device.max_ctp = DCMODE_EXT_MAX_SUBIF_PER_DEV;

        ret = dp_register_dev_ext(0, owner, port_id, &dp_cb, &dp_device, 0);
        if (ret != DP_SUCCESS) {
            DC_DP_ERROR("failed to register dev to Datapath Library/Core!!!\n");
            goto err_out_free_dev;
        }
    }

    /* For the first device */
    if ( (1 == dev_ctx->shared_info->ref_count) ) {
#if !defined(CONFIG_X86_INTEL_LGM) && !defined(CONFIG_PRX300_CQM)
        /* Setup Port resources */
        ret = dcmode_ext_setup_pmac_port(port_id, dev_ctx->shared_info->dma_cid, dev_ctx->shared_info->port_id, devspec->dev_cap_req, flags);
        if (ret != DP_SUCCESS) {
            DC_DP_ERROR("failed to setup UMT resources!!!\n");
            goto err_out_dereg_dev;
        }
#endif /* #if !defined(CONFIG_X86_INTEL_LGM) && !defined(CONFIG_PRX300_CQM) */

        /* Setup ring resources */
        DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "Preparing the resources to be returned for the port=%d.\n", port_id);
        ret = dcmode_ext_setup_ring_resources(dev_ctx, dev_ctx->shared_info->port_id, &dp_device, resources, flags);
        if (ret) {
            DC_DP_ERROR("failed to prepare the resources for the port_id=%d!!!\n", port_id);
            goto err_out_dereg_dev;
        }
    }

    devspec->dc_accel_used = DC_DP_ACCEL_FULL_OFFLOAD;
#if defined(CONFIG_X86_INTEL_LGM)
    devspec->dc_tx_ring_used = DC_DP_RING_HW_MODE1_EXT;
    devspec->dc_rx_ring_used = DC_DP_RING_HW_MODE1_EXT;
#elif defined(CONFIG_PRX300_CQM) /* #if defined(CONFIG_X86_INTEL_LGM) */
    devspec->dc_tx_ring_used = DC_DP_RING_HW_MODE0_EXT;
    devspec->dc_rx_ring_used = DC_DP_RING_HW_MODE0_EXT;
#else /* #elif defined(CONFIG_PRX300_CQM) */
    devspec->dc_tx_ring_used = DC_DP_RING_HW_MODE0;
    devspec->dc_rx_ring_used = DC_DP_RING_HW_MODE0;
#endif /* #else */
    get_soc_capability(&devspec->dc_cap);

    /* Register DC ModeX device to DC common layer */
    ret = dc_dp_register_dcmode_device(owner, port_id, dev, dev_ctx, 0);
    if (ret) {
        DC_DP_ERROR("failed to register device to DC common layer!!!\n");
        goto err_out_dereg_dev;
    }

    DC_DP_UNLOCK(&g_dcmode_ext_dev_lock);

    return ret;

err_out_dereg_dev:
    /* Skip, iff sub-sequent Multiport device? */
    if ( !is_multiport_sub(alloc_flags) )
        dp_register_dev_ext(0, owner, port_id, &dp_cb, &dp_device, DP_F_DEREGISTER);

err_out_free_dev:
    dcmode_ext_free_dev_ctx(dev_ctx);

err_unlock_out:
    DC_DP_UNLOCK(&g_dcmode_ext_dev_lock);

err_out:
    DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "Failure, returned %d.\n", ret);
    return ret;
}

static int32_t
dcmode_ext_register_subif(void *ctx,
                          struct module *owner, struct net_device *dev,
                          const uint8_t *subif_name, struct dp_subif *subif_id, uint32_t flags)
{
    int32_t ret = DC_DP_FAILURE;
    struct dcmode_ext_dev_info *dev_ctx = (struct dcmode_ext_dev_info *)ctx;
    struct dp_subif subif = {0};
    struct dp_subif_data subif_data = {0};

    DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "dev_ctx=%p, owner=%p, dev=%p, subif_id=%p, flags=0x%08X\n",
                    dev_ctx, owner, dev, subif_id, flags);

    memcpy(&subif, subif_id, sizeof(struct dp_subif));

    /* De-register */
    if (flags & DC_DP_F_DEREGISTER) {

        DC_DP_LOCK(&g_dcmode_ext_dev_lock);

        /* Multiport device? */
        if ( is_multiport(dev_ctx->alloc_flags) )
            multiport_wa_forward_map_subifid(dev_ctx, &subif);

        /* De-register subif from Datapath Library/Core */
        ret = dp_register_subif_ext(0, owner, dev, (uint8_t *)subif_name, &subif, &subif_data, DP_F_DEREGISTER);
        if (ret != DP_SUCCESS) {
            DC_DP_ERROR("failed to de-register subif from Datapath Library/Core!!!\n");
            goto err_unlock_out;
        }

        dev_ctx->num_subif--;
        dev_ctx->shared_info->num_subif--;
        dev_ctx->shared_info->subif_info[DCMODE_EXT_GET_SUBIFIDX(subif.subif)].netif = NULL;

        DC_DP_UNLOCK(&g_dcmode_ext_dev_lock);

        DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "Success, returned %d.\n", DC_DP_SUCCESS);
        return DC_DP_SUCCESS;
    }

    DC_DP_LOCK(&g_dcmode_ext_dev_lock);

    /* Multiport device? */
    if ( is_multiport(dev_ctx->alloc_flags) ) {
        /* Single port should not have more than 8 Subif */
        if ( DCMODE_EXT_GET_SUBIFIDX(subif_id->subif) >= MULTIPORT_WORKAROUND_MAX_SUBIF_NUM ) {
            DC_DP_ERROR("failed to register subif, subif_id_num=0x%04x reaches maximum limit 8!!!\n", subif_id->subif);
            goto err_unlock_out;
        }

        multiport_wa_forward_map_subifid(dev_ctx, &subif);
    }

    /* TODO : subif_data */
    //subif_data.deq_port_idx = ;
    //subif_data.flag_ops = ;

    /* Register subif to Datapath Library/Core */
    ret = dp_register_subif_ext(0, owner, dev, (uint8_t *)subif_name, &subif, &subif_data, 0);
    if (ret != DP_SUCCESS) {
        DC_DP_ERROR("failed to register subif to Datapath Library/Core!!!\n");
        goto err_unlock_out;
    }

    /* Multiport 1st device? */
    if ( is_multiport_main(dev_ctx->alloc_flags) )
        subif_id->subif = (subif.subif & ~MULTIPORT_WORKAROUND_SUBIFID_MASK);
    else
        subif_id->subif = subif.subif;

    dev_ctx->shared_info->subif_info[DCMODE_EXT_GET_SUBIFIDX(subif.subif)].netif = dev;
    dev_ctx->shared_info->num_subif++;
    dev_ctx->num_subif++;

    DC_DP_UNLOCK(&g_dcmode_ext_dev_lock);

    goto out;

err_unlock_out:
    DC_DP_UNLOCK(&g_dcmode_ext_dev_lock);

out:
    DC_DP_DEBUG(DC_DP_DBG_FLAG_DBG, "Returned %d.\n", ret);
    return ret;
}

static int32_t
dcmode_ext_xmit(void *ctx, struct net_device *rx_if, struct dp_subif *rx_subif, struct dp_subif *tx_subif,
                struct sk_buff *skb, int32_t len, uint32_t flags)
{
    int32_t ret = DC_DP_FAILURE;
    struct dp_subif subif = {0};
    uint32_t dp_flags = 0;
    struct dcmode_ext_dev_info *dev_ctx = (struct dcmode_ext_dev_info *)ctx;
#if !defined(CONFIG_X86_INTEL_LGM)
    struct dma_tx_desc_0 *desc_0 = (struct dma_tx_desc_0 *) &skb->DW0;
    struct dma_tx_desc_1 *desc_1 = (struct dma_tx_desc_1 *) &skb->DW1;
#endif /* #if !defined(CONFIG_X86_INTEL_LGM) */

    if (!tx_subif) {
        DC_DP_ERROR("tx_subif is NULL!!!\n");
        goto drop;
    }

    subif = *tx_subif;

    /* Multiport device? */
    if ( dev_ctx && is_multiport(dev_ctx->alloc_flags) )
        multiport_wa_forward_map_subifid(dev_ctx, &subif);

#if !defined(CONFIG_X86_INTEL_LGM)
    /* skb->DWx */
    desc_1->field.ep = subif.port_id;
    desc_0->field.dest_sub_if_id = subif.subif;
#endif /* #if !defined(CONFIG_X86_INTEL_LGM) */

    if ( (skb->ip_summed == CHECKSUM_PARTIAL) )
        dp_flags = DP_TX_CAL_CHKSUM;

    /* Send it to Datapath library for transmit */
    return dp_xmit(skb->dev, &subif, skb, skb->len, dp_flags);

drop:
    if (skb)
        dev_kfree_skb_any(skb);

    return ret;
}

#if (defined(DCMODE_EXT_BRIDGE_FLOW_LEARNING) && DCMODE_EXT_BRIDGE_FLOW_LEARNING)
static int32_t
dcmode_ext_add_session_shortcut_forward(void *ctx, struct dp_subif *subif, struct sk_buff *skb, uint32_t flags)
{
    int32_t ret = DC_DP_SUCCESS;

    /* FIXME : For LGM, need to review. */
#if !defined(CONFIG_X86_INTEL_LGM)
#if IS_ENABLED(CONFIG_PPA) && defined(CONFIG_PPA_BR_SESS_LEARNING)
    struct ethhdr *eth;

    if (!skb)
        return DC_DP_FAILURE;

    /* FIXME : Enabled bridge flow learning globally for all netif (mainly appicable for WLAN netif) */
    if ( (flags & DC_DP_F_PREFORWARDING) ) {
        skb_reset_mac_header(skb);
        eth = eth_hdr(skb);
        if (unlikely(is_multicast_ether_addr(eth->h_dest)) ||
            unlikely(ether_addr_equal_64bits(eth->h_dest, skb->dev->dev_addr))) {
            /* Skipping, as no acceleration is possible */
            return 0;
        }

        skb->pkt_type = PACKET_OTHERHOST;
        skb->protocol = ntohs(eth->h_proto);
        skb_set_network_header(skb, ETH_HLEN);

        if ( NULL != ppa_hook_session_add_fn )
            ret = ppa_hook_session_add_fn(skb, NULL, (PPA_F_BRIDGED_SESSION | PPA_F_BEFORE_NAT_TRANSFORM));
    } else if ( (flags & DC_DP_F_POSTFORWARDING) ) {
        if ( NULL != ppa_hook_session_add_fn )
            ret = ppa_hook_session_add_fn(skb, NULL, PPA_F_BRIDGED_SESSION);
    }
#endif /* #if IS_ENABLED(CONFIG_PPA) && defined(CONFIG_PPA_BR_SESS_LEARNING) */
#endif /* #if !defined(CONFIG_X86_INTEL_LGM) */

    return ret;
}
#endif /* #if (defined(DCMODE_EXT_BRIDGE_FLOW_LEARNING) && DCMODE_EXT_BRIDGE_FLOW_LEARNING) */

static int32_t
dcmode_ext_disconn_if(void *ctx, struct net_device *netif, struct dp_subif *subif_id,
                      uint8_t mac_addr[MAX_ETH_ALEN], uint32_t flags)
{
    int32_t ret = DC_DP_SUCCESS;

    /* FIXME : For LGM, need to review. */
#if !defined(CONFIG_X86_INTEL_LGM)
#if IS_ENABLED(CONFIG_PPA)
    struct dcmode_ext_dev_info *dev_ctx = (struct dcmode_ext_dev_info *)ctx;
    struct dp_subif subif = {0};

    subif = *subif_id;

    /* Multiport device? */
    if ( dev_ctx && is_multiport(dev_ctx->alloc_flags) )
        multiport_wa_forward_map_subifid(dev_ctx, &subif);

    /* Remove all the sessions from PPA */
    if (ppa_hook_disconn_if_fn)
        ret = ppa_hook_disconn_if_fn(netif, &subif, mac_addr, flags);
#endif /* #if IS_ENABLED(CONFIG_PPA) */
#endif /* #if !defined(CONFIG_X86_INTEL_LGM) */

    return ret;
}

/*
 * ========================================================================
 * Callbacks Registered to Datapath Library/Core
 * ========================================================================
 */
static int32_t
dcmode_ext_rx_cb(struct net_device *rxif, struct net_device *txif,
                 struct sk_buff *skb, int32_t len)
{
    int32_t ret;
#if defined(CONFIG_X86_INTEL_LGM)
    struct dma_rx_desc_0 *desc_0 = (struct dma_rx_desc_0 *)&skb->DW0;
#elif defined(CONFIG_PRX300_CQM)
    struct dma_rx_desc_1 *desc_1 = (struct dma_rx_desc_1 *)&skb->DW1;
#else /* #if defined(CONFIG_X86_INTEL_LGM) */
    struct pmac_rx_hdr *pmac;
#endif /* #else */
    struct dp_subif rx_subif = {0};
    struct dcmode_ext_dev_info *dev_ctx = NULL;

    if (!skb) {
        DC_DP_ERROR("failed to receive as skb=%p!!!\n", skb);
        goto err_out;
    }

    if (!rxif) {
        DC_DP_ERROR("failed to receive as rxif=%p!!!\n", rxif);
        goto err_out_drop;
    }

#if defined(CONFIG_X86_INTEL_LGM)
    ret = dcmode_ext_get_subif_by_dev(rxif, &rx_subif);
    if (ret != DC_DP_SUCCESS) {
        DC_DP_ERROR("Why <port_id=%d, subif_id=0x%x> for rxif=%s???\n",
                    rx_subif.port_id, rx_subif.subif, rxif->name);
        goto err_out_drop;
    }
    rx_subif.subif = desc_0->field.dest_sub_if_id;

#elif defined(CONFIG_PRX300_CQM) /* #if defined(CONFIG_X86_INTEL_LGM) */
    rx_subif.port_id = desc_1->field.ip;
    rx_subif.subif = desc_1->field.session_id;

    len -= sizeof(struct pmac_rx_hdr);
    skb_pull(skb, sizeof(struct pmac_rx_hdr));
#else /* #elif defined(CONFIG_PRX300_CQM) */
    pmac = (struct pmac_rx_hdr *)(skb->data);
    rx_subif.port_id = pmac->sppid;
    rx_subif.subif = (pmac->src_sub_inf_id << 8);
    rx_subif.subif |= (pmac->src_sub_inf_id2 & 0xFF);

    len -= sizeof(struct pmac_rx_hdr);
    skb_pull(skb, sizeof(struct pmac_rx_hdr));
#endif /* #else */

    dev_ctx = g_dcmode_ext_dev_p[rx_subif.port_id];
    if (NULL == dev_ctx) {
        DC_DP_ERROR("failed to receive as dev_ctx=%p for port_id=%d!!!\n", dev_ctx, rx_subif.port_id);
        goto err_out_drop;
    }

    /* Multiport device? */
    if ( is_multiport(dev_ctx->alloc_flags) )
        multiport_wa_reverse_map_subifid(dev_ctx, &rx_subif);

    if ( (rxif->features & NETIF_F_RXCSUM) )
        skb->ip_summed = CHECKSUM_UNNECESSARY;

    ret = dc_dp_rx(rxif, txif, &rx_subif, skb, skb->len, 0);

    return ret;

err_out_drop:
    dev_kfree_skb_any(skb);

err_out:
    return DP_FAILURE;
}

static int32_t
dcmode_ext_get_netif_subifid_cb(struct net_device *netif,
                                struct sk_buff *skb, void *subif_data,
                                uint8_t dst_mac[MAX_ETH_ALEN],
                                dp_subif_t *subif, uint32_t flags) /*! get subifid */
{
    int32_t ret = 1;
    struct dp_subif subif_id = {0};
    struct dcmode_ext_dev_info *dev_ctx;
    struct dcmode_ext_dev_shared_info *dev_shared_ctx;
    int32_t subif_idx;

    /* Validate input argument */
    if (!netif) {
        DC_DP_ERROR("failed to get subifid as netif=%p!!!\n", netif);
        goto out;
    }

    if (!subif) {
        DC_DP_ERROR("failed to get subifid as subif=%p!!!\n", subif);
        goto out;
    }

    /* Validate device context */
    dev_ctx = g_dcmode_ext_dev_p[subif->port_id];
    if (!dev_ctx) {
        DC_DP_ERROR("port_id=%d not registered!!!\n", subif->port_id);
        goto out;
    }

    dev_shared_ctx = dev_ctx->shared_info;
    if (!dev_shared_ctx) {
        DC_DP_ERROR("port_id=%d not registered!!!\n", subif->port_id);
        goto out;
    }

    /* FIXME : No valid subif from DPLib in get_subif_fn CB? */
    subif_idx = DCMODE_EXT_GET_SUBIFIDX(subif->subif);
    if (dev_shared_ctx->subif_info[subif_idx].netif != netif) {
        for (subif_idx = 0; subif_idx < DCMODE_EXT_MAX_SUBIF_PER_DEV; subif_idx++) {
            if (dev_shared_ctx->subif_info[subif_idx].netif == netif) {
                subif->subif &= ~(DCMODE_EXT_SUBIFID_MASK << DCMODE_EXT_SUBIFID_OFFSET);
                subif->subif |= (subif_idx << DCMODE_EXT_SUBIFID_OFFSET);
                break;
            }
        }

        if (subif_idx == DCMODE_EXT_MAX_SUBIF_PER_DEV) {
            DC_DP_ERROR("No matching netif=%p!!!\n", netif);
            goto out;
        }
    }

    memcpy(&subif_id, subif, sizeof(struct dp_subif));

    multiport_wa_reverse_map_subifid(dev_ctx, &subif_id);

    ret = dc_dp_get_netif_subifid(netif, skb, subif_data, dst_mac, &subif_id, flags);
    if (ret)
        goto out;

    subif->subif &= ~0x00FF;
    subif->subif |= (subif_id.subif & 0x00FF);

out:
    return ret;
}

/*
 * ========================================================================
 * Misclleneous API
 * ========================================================================
 */

static int32_t
dcmode_ext_map_class2devqos(void *ctx, int32_t port_id, struct net_device *netif,
                            uint8_t class2prio[], uint8_t prio2devqos[])
{
    /* FIXME : For LGM, is it still required? */
#if !defined(CONFIG_X86_INTEL_LGM) && !defined(CONFIG_PRX300_CQM)
    uint8_t devqos;

    /* Configure the egress PMAC table to mark the WMM/TID in the descriptor DW1[7:4] */
    uint8_t i = 0, j = 0;
    GSW_PMAC_Eg_Cfg_t egcfg;
    GSW_API_HANDLE gswr;

    struct dcmode_ext_dev_info *dev_ctx = (struct dcmode_ext_dev_info *)ctx;

    /* Multiport sub-sequent device? */
    if ( dev_ctx && is_multiport_sub(dev_ctx->alloc_flags) )
        return DC_DP_SUCCESS;

    /* Do the GSW-R configuration */
    gswr = gsw_api_kopen(SWITCH_DEV);
    if (gswr == 0) {
        DC_DP_ERROR("failed to open SWAPI device!!!\n");
        return -EIO;
    }

    /* GSWIP-R PMAC Egress Configuration Table */
    for (i = 0; i < DC_DP_MAX_SOC_CLASS; i++) {
        devqos = _dc_dp_get_class2devqos(class2prio, prio2devqos, i);

        for (j = 0; j <= 3; j++) {
            memset((void *)&egcfg, 0x00, sizeof(egcfg));
            egcfg.nDestPortId   = port_id;
            egcfg.nTrafficClass = i;
            egcfg.nFlowIDMsb    = j;
            gsw_api_kioctl(gswr, GSW_PMAC_EG_CFG_GET, (unsigned int)&egcfg);

            egcfg.nResDW1       = devqos;
            gsw_api_kioctl(gswr, GSW_PMAC_EG_CFG_SET, (unsigned int)&egcfg);
        }
    }

    gsw_api_kclose(gswr);
#endif /* #if !defined(CONFIG_X86_INTEL_LGM) && !defined(CONFIG_PRX300_CQM) */

    return DC_DP_SUCCESS;
}

static int32_t
dcmode_ext_get_netif_stats(void *ctx,
                           struct net_device *netif, struct dp_subif *subif_id,
                           struct rtnl_link_stats64 *if_stats, uint32_t flags)
{
    //int32_t ret;
    uint32_t dp_flags = 0x0;
    struct dcmode_ext_dev_info *dev_ctx = (struct dcmode_ext_dev_info *)ctx;
    struct dp_subif subif = {0};

    subif = *subif_id;

    /* Multiport radio? */
    if ( dev_ctx && is_multiport(dev_ctx->alloc_flags) )
        multiport_wa_forward_map_subifid(dev_ctx, &subif);

    if ( (flags & DC_DP_F_SUBIF_LOGICAL) )
        dp_flags = DP_F_STATS_SUBIF;

    return dp_get_netif_stats(netif, &subif, if_stats, dp_flags);
}

static int32_t
dcmode_ext_clear_netif_stats(void *ctx,
                             struct net_device *netif, struct dp_subif *subif_id,
                             uint32_t flags)
{
    uint32_t dp_flags = 0x0;
    struct dcmode_ext_dev_info *dev_ctx = (struct dcmode_ext_dev_info *)ctx;
    struct dp_subif subif = {0};

    subif = *subif_id;

    /* Multiport radio? */
    if ( dev_ctx && is_multiport(dev_ctx->alloc_flags) )
        multiport_wa_forward_map_subifid(dev_ctx, &subif);

    if ( (flags & DC_DP_F_SUBIF_LOGICAL) )
        dp_flags = DP_F_STATS_SUBIF;

    return dp_clear_netif_stats(netif, &subif, dp_flags);
}

static void
dcmode_ext_dump_proc(void *ctx, struct seq_file *seq)
{
    struct dcmode_ext_dev_info *dev_ctx = (struct dcmode_ext_dev_info *)ctx;

    if (!dev_ctx)
        return;

    seq_printf(seq, "    cqm_pid:           %d\n",
           dev_ctx->shared_info->cqm_pid);

    seq_printf(seq, "    dma_ch:            %d\n",
           dev_ctx->shared_info->dma_ch);

    seq_printf(seq, "    num_bufpools:      %02d\n",
           dev_ctx->shared_info->num_bufpools);

    seq_printf(seq, "    umt_id:            %d\n",
           dev_ctx->shared_info->umt_id);
    seq_printf(seq, "    umt_period:        %d (in micro second)\n",
           dev_ctx->shared_info->umt_period);
}

static struct dc_dp_dcmode_ops dcmode_ext_ops = {
    .get_host_capability = dcmode_ext_get_host_capability,
    .register_dev = NULL,
    .register_dev_ex = dcmode_ext_register_dev_ex,
    .register_subif = dcmode_ext_register_subif,
    .xmit = dcmode_ext_xmit,
    .handle_ring_sw = NULL,
    .add_session_shortcut_forward = dcmode_ext_add_session_shortcut_forward,
    .disconn_if = dcmode_ext_disconn_if,
    .get_netif_stats = dcmode_ext_get_netif_stats,
    .clear_netif_stats = dcmode_ext_clear_netif_stats,
    .register_qos_class2prio_cb = NULL,
    .map_class2devqos = dcmode_ext_map_class2devqos,
    .alloc_skb = NULL,
    .free_skb = NULL,
    .change_dev_status = NULL,
    .get_wol_cfg = NULL,
    .set_wol_cfg = NULL,
    .get_wol_ctrl_status = NULL,
    .set_wol_ctrl = NULL,
    .dump_proc = dcmode_ext_dump_proc
};

static struct dc_dp_dcmode dcmode_ext = {
    .dcmode_cap = DC_DP_F_DCMODE_HW | DC_DP_F_DCMODE_0,
    .dcmode_ops = &dcmode_ext_ops
};

#if !IS_ENABLED(CONFIG_X86_INTEL_LGM)
#if IS_ENABLED(CONFIG_PPA)
static int ppa_class2prio_event(PPA_NOTIFIER_BLOCK *nb,
                                unsigned long action, void *ptr)
{
    struct ppa_class2prio_notifier_info *info;

    switch (action) {
    case PPA_CLASS2PRIO_DEFAULT:
    case PPA_CLASS2PRIO_CHANGE:
        info = (struct ppa_class2prio_notifier_info *)ptr;
        if (info)
            dc_dp_qos_class2prio(info->port_id, info->dev, info->class2prio);
        break;
    default:
        break;
    }

    return PPA_NOTIFY_OK;
}

PPA_NOTIFIER_BLOCK ppa_class2prio_notifier = {
	.notifier_call = ppa_class2prio_event
};
#endif
#endif

static __init int dcmode_ext_init_module(void)
{
    int32_t ret = 0;

    if (!g_dcmode_ext_init_ok) {
        /* Get Platform capability */
        ret = dp_get_cap(&g_dp_cap, 0);
        if (ret)
            DC_DP_ERROR("failed to get DP capability!!!\n");

        memset(g_dcmode_ext_dev, 0, sizeof(g_dcmode_ext_dev));

        /* Register DCMODE */
        ret = dc_dp_register_dcmode(&dcmode_ext, 0);

#if IS_ENABLED(CONFIG_PPA)
        ppa_check_if_netif_fastpath_fn = dc_dp_check_if_netif_fastpath;
#if !IS_ENABLED(CONFIG_X86_INTEL_LGM)
        ppa_register_event_notifier(&ppa_class2prio_notifier);
#endif
#endif /* #if IS_ENABLED(CONFIG_PPA) */

        g_dcmode_ext_init_ok = 1;
    }

    return ret;
}

static __exit void dcmode_ext_exit_module(void)
{
    if (g_dcmode_ext_init_ok) {
#if IS_ENABLED(CONFIG_PPA)
#if !IS_ENABLED(CONFIG_X86_INTEL_LGM)
        ppa_unregister_event_notifier(&ppa_class2prio_notifier);
#endif
        ppa_check_if_netif_fastpath_fn = NULL;
#endif /* #if IS_ENABLED(CONFIG_PPA) */

        /* De-register DCMODE */
        dc_dp_register_dcmode(&dcmode_ext, DC_DP_F_DCMODE_DEREGISTER);

        /* Reset private data structure */
        memset(g_dcmode_ext_dev, 0, sizeof(g_dcmode_ext_dev));
        g_dcmode_ext_init_ok = 0;
    }
}

module_init(dcmode_ext_init_module);
module_exit(dcmode_ext_exit_module);

MODULE_AUTHOR("Anath Bandhu Garai");
MODULE_DESCRIPTION("Extended DC Mode support for any platform.");
MODULE_LICENSE("GPL");
MODULE_VERSION(DCMODE_EXT_DRV_MODULE_VERSION);
