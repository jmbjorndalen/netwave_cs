/*********************************************************************
 *                
 * Filename:      netwavep_cs.c
 * Version:       0.4.1
 * Description:   Netwave AirSurfer Wireless LAN PC Card driver
 * Status:        Hack
 * Authors:       John Markus Bjørndalen <johnm@cs.uit.no>
 *                Dag Brattli <dagb@cs.uit.no>
 *                David Hinds <dhinds@hyper.stanford.edu>
 * Created at:    A long time ago!
 * Modified at:   Mon Nov 10 11:54:37 1997
 * Modified by:   Dag Brattli <dagb@cs.uit.no>
 * 
 *     Copyright (c) 1997 University of Tromsø, Norway
 *
 * 
 * WARNING: This file is currently just "hacked up" quality. 
 *
 ********************************************************************/

/* To have statistics (just packets sent) define this */
#undef NETWAVEP_STATS

#include <pcmcia/config.h>
#include <pcmcia/k_compat.h>

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>
#include <linux/in.h>
#include <linux/malloc.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <asm/system.h>
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <linux/errno.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#ifdef HAS_WIRELESS_EXTENSIONS
#include <linux/wireless.h>
#endif

#include <pcmcia/version.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/cisreg.h>
#include <pcmcia/ds.h>
#include <pcmcia/mem_op.h>

/* 
 * Offsets for the Netwave Plus adapters
 */
#define NWP_LCTD   0x3ec  /* Last Completed Tx Desc  */
#define NWP_RES    0x400  /* Reserved                */
#define NWP_BANN   0x480  /* Banner                  */
#define NWP_CMD    0x4a0  /* Command block           */
#define NWP_CSB    0x4f0  /* Control/Status block    */
#define NWP_VBM    0x500  /* Virtual Bit Map         */
#define NWP_BUF    0x600  /* Buffer Space            */

/*
 * Netwave Plus commands. 
 */
#define NWP_CMD_TEST     0x01   /* Test Interface/NOP */
#define NWP_CMD_SMIB     0x02   /* Set Mib Variables  */
#define NWP_CMD_INITX    0x03   /* Init_Tx            */
#define NWP_CMD_FLUTX    0x04   /* Flush_Tx           */
#define NWP_CMD_ENRX     0x05   /* Enable Receiver    */
#define NWP_CMD_DIRX     0x06   /* Disable Receiver   */
#define NWP_CMD_SLEEP    0x07   /* Sleep              */
#define NWP_CMD_WAKE     0x08   /* Wake               */
#define NWP_CMD_GMIB     0x09   /* Get_MIB_Variables  */
#define NWP_CMD_SCAN     0x0a   /* Scan               */
#define NWP_CMD_SYNC     0x0b   /* Sync               */
#define NWP_CMD_RES      0x0c   /* Resume             */

/* 
 * Netwave Plus Command Results. 
 */ 
#define NWP_CRES_IDLE   0x00   /* Idle */
#define NWP_CRES_COMP   0x01   /* Command completed */
#define NWP_CRES_REJ_U  0x02   /* Command rejected - Unknown command */
#define NWP_CRES_REJ_I  0x03   /* Command rejected - Invalid parameter */
#define NWP_CRES_REJ_F  0x04   /* Command rejected - Function not supported */
#define NWP_CRES_REJ_R  0x05   /* Command rejected - Invalid result */
#define NWP_CRES_REJ_V  0x06   /* Command rejected - Function not supported */


#define NETWAVEP_REGOFF         0x8000
/* The Netwave IO registers, offsets to iobase */
#define NETWAVEP_REG_COR        0x0
#define NETWAVEP_REG_CCSR       0x2
#define NETWAVEP_REG_ASR        0x4
#define NETWAVEP_REG_IMR        0xa
#define NETWAVEP_REG_PMR        0xc
#define NETWAVEP_REG_IOLOW      0x6
#define NETWAVEP_REG_IOHI       0x7
#define NETWAVEP_REG_IOCONTROL  0x8
#define NETWAVEP_REG_DATA       0xf
/* The Netwave Extended IO registers, offsets to RamBase */
#define NETWAVEP_EREG_ASCC      0x114
#define NETWAVEP_EREG_RSER      0x120
#define NETWAVEP_EREG_RSERW     0x124
#define NETWAVEP_EREG_TSER      0x130
#define NETWAVEP_EREG_TSERW     0x134
#define NETWAVEP_EREG_CB        0x100
#define NETWAVEP_EREG_SPCQ      0x154
#define NETWAVEP_EREG_SPU       0x155
#define NETWAVEP_EREG_LIF       0x14e
#define NETWAVEP_EREG_ISPLQ     0x156
#define NETWAVEP_EREG_HHC       0x158
#define NETWAVEP_EREG_NI        0x16e
#define NETWAVEP_EREG_MHS       0x16b
#define NETWAVEP_EREG_TDP       0x140
#define NETWAVEP_EREG_RDP       0x150
#define NETWAVEP_EREG_PA        0x160
#define NETWAVEP_EREG_EC        0x180
#define NETWAVEP_EREG_CRBP      0x17a
#define NETWAVEP_EREG_ARW       0x166

/*
 * Commands used in the extended command buffer
 * NETWAVEP_EREG_CB (0x100-0x10F) 
 */
#define NETWAVEP_CMD_NOP        0x00
#define NETWAVEP_CMD_SRC        0x01
#define NETWAVEP_CMD_STC        0x02
#define NETWAVEP_CMD_AMA        0x03
#define NETWAVEP_CMD_DMA        0x04
#define NETWAVEP_CMD_SAMA       0x05
#define NETWAVEP_CMD_ER         0x06
#define NETWAVEP_CMD_DR         0x07
#define NETWAVEP_CMD_TL         0x08
#define NETWAVEP_CMD_SRP        0x09
#define NETWAVEP_CMD_SSK        0x0a
#define NETWAVEP_CMD_SMD        0x0b
#define NETWAVEP_CMD_SAPD       0x0c
#define NETWAVEP_CMD_SSS        0x11
/* End of Command marker */
#define NETWAVEP_CMD_EOC        0x00

/* ASR register bits */
#define NETWAVEP_ASR_RXRDY   0x80
#define NETWAVEP_ASR_TXBA    0x01

#define TX_TIMEOUT  20
#define WATCHDOG_JIFFIES 32

static const unsigned int imrConfRFU1 = 0x10; /* RFU interrupt mask, keep high */
static const unsigned int imrConfIENA = 0x02; /* Interrupt enable */

static const unsigned int corConfIENA   = 0x01; /* Interrupt enable */
static const unsigned int corConfLVLREQ = 0x40; /* Keep high */

static const unsigned int rxConfRxEna  = 0x80; /* Receive Enable */
static const unsigned int rxConfMAC    = 0x20; /* MAC host receive mode*/ 
static const unsigned int rxConfPro    = 0x10; /* Promiscuous */
static const unsigned int rxConfAMP    = 0x08; /* Accept Multicast Packets */
static const unsigned int rxConfBcast  = 0x04; /* Accept Broadcast Packets */

static const unsigned int txConfTxEna  = 0x80; /* Transmit Enable */
static const unsigned int txConfMAC    = 0x20; /* Host sends MAC mode */
static const unsigned int txConfEUD    = 0x10; /* Enable Uni-Data packets */
static const unsigned int txConfKey    = 0x02; /* Scramble data packets */
static const unsigned int txConfLoop   = 0x01; /* Loopback mode */

/*static int netwavep_debug = 0;*/

/*
   All the PCMCIA modules use PCMCIA_DEBUG to control debugging.  If
   you do not define PCMCIA_DEBUG at all, all the debug code will be
   left out.  If you compile with PCMCIA_DEBUG=0, the debug code will
   be present but disabled -- but it can then be enabled for specific
   modules at load time with a 'pc_debug=#' option to insmod.
*/

#ifdef PCMCIA_DEBUG
static int pc_debug = PCMCIA_DEBUG;
MODULE_PARM(pc_debug, "i");
#define DEBUG(n, args...) if (pc_debug>(n)) printk(KERN_DEBUG args)
static char *version =
"netwavep_cs.c 0.3.0 Thu Jul 17 14:36:02 1997 (John Markus Bjørndalen)\n";
#else
#define DEBUG(n, args...)
#endif

static dev_info_t dev_info = "netwavep_cs";

/*====================================================================*/

/* Parameters that can be set with 'insmod' */

/* Choose the domain, default is 0x100 */
static u_int  domain = 0x100;

/* Scramble key, range from 0x0 to 0xffff.  
 * 0x0 is no scrambling. 
 */
static u_int  scramble_key = 0x0;

/* Shared memory speed, in ns. The documentation states that 
 * the card should not be read faster than every 400ns. 
 * This timing should be provided by the HBA. If it becomes a 
 * problem, try setting mem_speed to 400. 
 */
static int mem_speed = 0;

/* Bit map of interrupts to choose from */
/* This means pick from 15, 14, 12, 11, 10, 9, 7, 5, 4, and 3 */
static u_int irq_mask = 0xdeb8;
static int irq_list[4] = { -1 };

MODULE_PARM(domain, "i");
MODULE_PARM(scramble_key, "i");
MODULE_PARM(mem_speed, "i");
MODULE_PARM(irq_mask, "i");
MODULE_PARM(irq_list, "1-4i");

/*====================================================================*/

/* PCMCIA (Card Services) related functions */
static void netwavep_release(u_long arg);     /* Card removal */
static int  netwavep_event(event_t event, int priority, 
					      event_callback_args_t *args);
static void netwavep_pcmcia_config(dev_link_t *arg); /* Runs after card 
													   insertion */
static dev_link_t *netwavep_attach(void);     /* Create instance */
static void netwavep_detach(dev_link_t *);    /* Destroy instance */

/* Hardware configuration */
static void netwavep_doreset(unsigned long iobase, u_char* ramBase);
static void netwavep_reset(struct device *dev);

/* Misc device stuff */
static int netwavep_open(struct device *dev);  /* Open the device */
static int netwavep_close(struct device *dev); /* Close the device */
static int netwavep_config(struct device *dev, struct ifmap *map);

/* Packet transmission and Packet reception */
static int netwavep_start_xmit( struct sk_buff *skb, struct device *dev);
static int netwavep_rx( struct device *dev);

/* Interrupt routines */
static void netwavep_interrupt IRQ(int irq, void *dev_id, struct pt_regs *regs);
static void netwavep_watchdog(u_long);	/* Transmission watchdog */

/* Statistics */
static void update_stats(struct device *dev);
static struct enet_statistics *netwavep_get_stats(struct device *dev);

/* Wireless extensions */
#ifdef WIRELESS_EXT
static struct iw_statistics* netwavep_get_wireless_stats(struct device *dev);
#endif
static int netwavep_ioctl(struct device *, struct ifreq *, int);

#ifdef NEW_MULTICAST
static void set_multicast_list(struct device *dev);
#else
static void set_multicast_list(struct device *dev, int num_addrs, void *addrs);
#endif


/*
   A linked list of "instances" of the skeleton device.  Each actual
   PCMCIA card corresponds to one device instance, and is described
   by one dev_link_t structure (defined in ds.h).

   You may not want to use a linked list for this -- for example, the
   memory card driver uses an array of dev_link_t pointers, where minor
   device numbers are used to derive the corresponding array index.
*/
static dev_link_t *dev_list = NULL;

/*
   A dev_link_t structure has fields for most things that are needed
   to keep track of a socket, but there will usually be some device
   specific information that also needs to be kept track of.  The
   'priv' pointer in a dev_link_t structure can be used to point to
   a device-specific private data structure, like this.

   A driver needs to provide a dev_node_t structure for each device
   on a card.  In some cases, there is only one device per card (for
   example, ethernet cards, modems).  In other cases, there may be
   many actual or logical devices (SCSI adapters, memory cards with
   multiple partitions).  The dev_node_t structures need to be kept
   in a linked list starting at the 'dev' field of a dev_link_t
   structure.  We allocate them in the card's private data structure,
   because they generally can't be allocated dynamically.
*/

#define SIOCGIPSNAP	SIOCDEVPRIVATE		/* Site Survey Snapshot */
/*#define SIOCGIPQTHR	SIOCDEVPRIVATE + 1*/

#define MAX_ESA 10

typedef struct net_addr {
    u_char addr48[6];
} net_addr;

struct site_survey {
    u_short length;
    u_char  struct_revision;
    u_char  roaming_state;
	
    u_char  sp_existsFlag;
    u_char  sp_link_quality;
    u_char  sp_max_link_quality;
    u_char  linkQualityGoodFairBoundary;
    u_char  linkQualityFairPoorBoundary;
    u_char  sp_utilization;
    u_char  sp_goodness;
    u_char  sp_hotheadcount;
    u_char  roaming_condition;
	
    net_addr sp;
    u_char   numAPs;
    net_addr nearByAccessPoints[MAX_ESA];
};	
   
typedef struct netwavep_private {
    dev_node_t node;
    u_char     *ramBase;
    int        timeoutCounter;
    int        lastExec;
    struct timer_list      watchdog;	/* To avoid blocking state */
    struct site_survey     nss;
    struct enet_statistics stats;
#ifdef WIRELESS_EXT
    struct iw_statistics   iw_stats;    /* Wireless stats */
#endif
} netwavep_private;




/* 
 * These structures have their field names from the 
 * api documentation. 
 */

/* Last_Completed_Descriptor_Block */
typedef struct NWP_LCD_Block 
{
    u_long Last_Bcast_Tx_Desc;  
    u_long Last_Mgmt_Tx_Desc;
    u_long Last_Data_Tx_Desc;
    u_long Last_PS_Poll_Tx_Desc;
    u_long Last_CF_Poll_Tx_Desc;
} NWP_LCD_Block;

/* Command_Block 
 * 
 *  The driver will set the Command byte to a non-zero value. Once the
 *  PCnet-MOBILE firmware has completed the command, it will set the
 *  Command Status to a non-zero value indicating to the driver that
 *  the command is complete. Once the driver has read the
 *  Command_Status it should first clear the Command byte and then the
 *  Command_Status byte.
 */
typedef struct NWP_C_Block
{
    u_char Command;
    u_char Command_Status;
    u_char Error_Offset;
    u_char Reserved;
    u_char Command_Parameters[76];
} NWP_C_Block;




typedef struct NWP_ConStat_Block {
    u_char Self_Test_Status;		/* Read only */
    u_char STA_State;                   /* Read only */
    u_char Rsvd_For_User_Rtn;
    u_char Interrupt_status;
    u_char Interrupt_Mask;
    u_char Lockout_PCnet_MOBILE;
    u_char Lockout_Host;                /* Read only */
    u_char Interrupt_Status2;
    u_long Reserved;
    u_char SW_Dis_Pwr_Dn;
    u_char Interrupt_Mask2;
    u_char Driver_State;
    u_char Reserved2;
} NWP_ConStat_Block;


typedef struct NWP_GMIB_Variables {
    u_char         Type;
    u_char         Size;
    u_char         Index;
    u_char         Reserved;
    char          Data[72];
} NWP_GMIB_Variables;


typedef struct NWP_LMIB_Struct {
    u_char       Fragmentation_Dis;
    u_char       Add_PLCP_Dis;
    u_char       MAC_Hdr_Prsv;
    u_char       Rx_Mgmt_Que_En;

    u_char       Re_Assembly_Dis;
    u_char       Strip_PLCP_Dis;
    u_char       Rx_Error_Dis;
    u_char       Power_Saving_Mode_Dis;

    u_char       Accept_All_Multicast_Dis;
    u_char       Check_Seq_Cntl_Dis;
    u_char       Flush_CFP_Queue_On_CF_End;
    u_char       Network_Mode;

    u_char       PWD_Lvl;
    u_char       CFP_Mode;

    u_long       Tx_Buffer_Offset;
    u_long       Tx_Buffer_Size;
    u_long       Rx_Buffer_Offset;
    u_long       Rx_Buffer_Size;

    u_char       Acting_as_AP;
    u_char       Fill_CFP;
} __attribute__ ((packed)) NWP_LMIB_Type ;

typedef struct NWP_Init_Tx_Struct
{
    u_long           Data_Desc;
    u_long           Mgmt_Desc;
    u_long           Broadcast_Desc;
    u_long           PS_Poll_Desc;
    u_long           CF_Poll_Desc;
} NWP_Init_Tx_Type;


typedef struct NWP_Enable_Receiver_Struct
{
    u_long          Data_Desc;
    u_long          PS_Poll_Mgmt_Desc;
} NWP_Enable_Receiver_Type;

NWP_LMIB_Type local_mib;


#ifdef NETWAVEP_STATS
static struct enet_statistics *netwavep_get_stats(struct device *dev);
#endif

/*
 * The Netwave card is little-endian, so won't work for big endian
 * systems.
 */
static inline unsigned short get_uint16(u_char* staddr) 
{
    return readw(staddr); /* Return only 16 bits */
}

static inline short get_int16(u_char* staddr)
{
    return readw(staddr);
}

/**************************************************************************/

static void cs_error(client_handle_t handle, int func, int ret)
{
    error_info_t err = { func, ret };
    CardServices(ReportError, handle, &err);
}

/* 
 * Wait until the WOC (Write Operation Complete) bit in the 
 * ASR (Adapter Status Register) is asserted. 
 * This should have aborted if it takes too long time. 
 */
static inline void wait_WOC(unsigned int iobase)
{
    /* Spin lock */
    //    while ((inb(iobase + NETWAVEP_REG_ASR) & 0x8) != 0x8) ; 
}

/* 
 * Wait until the Command Status flag ends up non-zero. 
 */
static inline u_char wait_CStatus(unsigned char* ramBase)
{
    u_char r;
    int c = 0;

    while ((r = readb(ramBase + NWP_CMD + 1)) == 0 && c++ < 10000);

    return r;
}

#ifdef WIRELESS_EXT
static void netwavep_snapshot(netwavep_private *priv, u_char *ramBase, 
			     unsigned short iobase) { 
    u_short resultBuffer;

    /* if time since last snapshot is > 1 sec. (100 jiffies?)  then take 
     * new snapshot, else return cached data. This is the recommended rate.  
     */
    if ( jiffies - priv->lastExec > 100) { 
	/* Take site survey  snapshot */ 
	/*printk( KERN_DEBUG "Taking new snapshot. %ld\n", jiffies -
	  priv->lastExec); */
	wait_WOC(iobase); 
	//writeb(NETWAVEP_CMD_SSS, ramBase + NETWAVEP_EREG_CB + 0); 
	//writeb(NETWAVEP_CMD_EOC, ramBase + NETWAVEP_EREG_CB + 1); 
	wait_WOC(iobase); 

	/* Get result and copy to cach */ 
	resultBuffer = readw(ramBase + NETWAVEP_EREG_CRBP); 
	copy_from_pc( &priv->nss, ramBase+resultBuffer, 
		      sizeof(struct site_survey)); 
    } 
}
#endif

#ifdef WIRELESS_EXT
/*
 * Function netwavep_get_wireless_stats (dev)
 *
 *    Wireless extensions statistics
 *
 */
static struct iw_statistics *netwavep_get_wireless_stats(struct device *dev)
{	
    unsigned long flags;
    unsigned short iobase = dev->base_addr;
    netwavep_private *priv = (netwavep_private *) dev->priv;
    u_char *ramBase = priv->ramBase;
    struct iw_statistics* wstats;
	
    wstats = &priv->iw_stats;

    save_flags(flags);
    cli();
	
    netwavep_snapshot( priv, ramBase, iobase);

    wstats->status = priv->nss.roaming_state;
    wstats->qual.qual = readb( ramBase + NETWAVEP_EREG_SPCQ); 
    wstats->qual.level = readb( ramBase + NETWAVEP_EREG_ISPLQ);
    wstats->qual.noise = readb( ramBase + NETWAVEP_EREG_SPU) & 0x3f;
    wstats->discard.nwid = 0L;
    wstats->discard.code = 0L;
    wstats->discard.misc = 0L;

    restore_flags(flags);
    
    return &priv->iw_stats;
}
#endif

/*
 * Function netwavep_init (dev)
 *
 *    We never need to do anything when a device is "initialized"
 *    by the net software, because we only register already-found cards.
 */
int netwavep_init(struct device *dev)
{
    /* We do all the initialization of this in netwavep_attach instead */
    return 0;
}

/*
 * Function netwavep_attach (void)
 *
 *     Creates an "instance" of the driver, allocating local data 
 *     structures for one device.  The device is registered with Card 
 *     Services.
 *
 *     The dev_link structure is initialized, but we don't actually
 *     configure the card at this point -- we wait until we receive a
 *     card insertion event.
 */
static dev_link_t *netwavep_attach(void)
{
    client_reg_t client_reg;
    dev_link_t *link;
    struct device *dev;
    netwavep_private *priv; 
    int i, ret;
    
    DEBUG(0, "netwavep_attach()\n");
    
    /* Initialize the dev_link_t structure */
    link = kmalloc(sizeof(struct dev_link_t), GFP_KERNEL);
    memset(link, 0, sizeof(struct dev_link_t));
    link->release.function = &netwavep_release;
    link->release.data = (u_long)link;
	
    /* The io structure describes IO port mapping */
    link->io.NumPorts1 = 16;
    link->io.Attributes1 = IO_DATA_PATH_WIDTH_16;
    /* link->io.NumPorts2 = 16; 
       link->io.Attributes2 = IO_DATA_PATH_WIDTH_16; */
    link->io.IOAddrLines = 5;
    
    /* Interrupt setup */
    link->irq.Attributes = IRQ_TYPE_EXCLUSIVE | IRQ_HANDLE_PRESENT;
    link->irq.IRQInfo1 = IRQ_INFO2_VALID|IRQ_LEVEL_ID;
    if (irq_list[0] == -1)
	link->irq.IRQInfo2 = irq_mask;
    else
	for (i = 0; i < 4; i++)
	    link->irq.IRQInfo2 |= 1 << irq_list[i];
    link->irq.Handler = &netwavep_interrupt;
    
    /* General socket configuration */
    link->conf.Attributes = CONF_ENABLE_IRQ;
    link->conf.Vcc = 50;
    link->conf.IntType = INT_MEMORY_AND_IO;
    link->conf.ConfigIndex = 1;
    link->conf.Present = PRESENT_OPTION;

    /* Allocate space for private device-specific data */
    dev = kmalloc(sizeof(struct device), GFP_KERNEL);
    memset(dev, 0, sizeof(struct device));

    dev->priv = kmalloc(sizeof(netwavep_private), GFP_KERNEL);
    memset(dev->priv, 0, sizeof(netwavep_private));

    /* Set the watchdog timer */
    priv = (netwavep_private *) dev->priv;
    priv->watchdog.function = &netwavep_watchdog;
    priv->watchdog.data = (unsigned long) dev;

    /* Netwave specific entries in the device structure */
    dev->hard_start_xmit = &netwavep_start_xmit;
    dev->set_config = &netwavep_config;
    dev->get_stats  = &netwavep_get_stats;
    dev->set_multicast_list = &set_multicast_list;
    /* wireless extensions */
#ifdef WIRELESS_EXT
    dev->get_wireless_stats = &netwavep_get_wireless_stats;
#endif
    dev->do_ioctl = &netwavep_ioctl;

    ether_setup(dev);
    dev->name = ((struct netwavep_private *)dev->priv)->node.dev_name;
    dev->init = &netwavep_init;
    dev->open = &netwavep_open;
    dev->stop = &netwavep_close;
    dev->tbusy = 1;
    link->priv = link->irq.Instance = dev;
    
    /* Register with Card Services */
    link->next = dev_list;
    dev_list = link;
    client_reg.dev_info = &dev_info;
    client_reg.Attributes = INFO_IO_CLIENT | INFO_CARD_SHARE;
    client_reg.EventMask =
	CS_EVENT_CARD_INSERTION | CS_EVENT_CARD_REMOVAL |
	CS_EVENT_RESET_PHYSICAL | CS_EVENT_CARD_RESET |
	CS_EVENT_PM_SUSPEND | CS_EVENT_PM_RESUME;
    client_reg.event_handler = &netwavep_event;
    client_reg.Version = 0x0210;
    client_reg.event_callback_args.client_data = link;
    ret = CardServices(RegisterClient, &link->handle, &client_reg);
    if (ret != 0) {
	cs_error(link->handle, RegisterClient, ret);
	netwavep_detach(link);
	return NULL;
    }

    return link;
} /* netwavep_attach */

/*
 * Function netwavep_detach (link)
 *
 *    This deletes a driver "instance".  The device is de-registered
 *    with Card Services.  If it has been released, all local data
 *    structures are freed.  Otherwise, the structures will be freed
 *    when the device is released.
 */
static void netwavep_detach(dev_link_t *link)
{
    dev_link_t **linkp;
    long flags;

    DEBUG(0, "netwavep_detach(0x%p)\n", link);
  
    /* Locate device structure */
    for (linkp = &dev_list; *linkp; linkp = &(*linkp)->next)
	if (*linkp == link) break;
    if (*linkp == NULL)
	return;

    save_flags(flags);
    if (link->state & DEV_RELEASE_PENDING) {
	del_timer(&link->release);
	link->state &= ~DEV_RELEASE_PENDING;
    }
    cli();
    restore_flags(flags);

    /*
	  If the device is currently configured and active, we won't
	  actually delete it yet.  Instead, it is marked so that when
	  the release() function is called, that will trigger a proper
	  detach().
	*/
    if (link->state & DEV_CONFIG) {
	netwavep_release((u_long) link);
	if (link->state & DEV_STALE_CONFIG) {
	    DEBUG(1, "netwavep_cs: detach postponed, '%s' still "
		  "locked\n", link->dev->dev_name);
	    
	    link->state |= DEV_STALE_LINK;
	    return;
	}
    }
	
    /* Break the link with Card Services */
    if (link->handle)
	CardServices(DeregisterClient, link->handle);
    
    /* Unlink device structure, free pieces */
    *linkp = link->next;
    if (link->priv) {
	struct device *dev = link->priv;
	if (dev->priv)
	    kfree_s(dev->priv, sizeof(netwavep_private));
	kfree_s(link->priv, sizeof(struct device));
    }
    kfree_s(link, sizeof(struct dev_link_t));
    
} /* netwavep_detach */

/*
 * Function netwavep_ioctl (dev, rq, cmd)
 *
 *     Perform ioctl : config & info stuff
 *     This is the stuff that are treated the wireless extensions (iwconfig)
 *
 */
static int netwavep_ioctl(struct device *dev, /* ioctl device */
						 struct ifreq *rq,	 /* Data passed */
						 int	cmd)	     /* Ioctl number */
{
    unsigned long flags;
    int			ret = 0;
#ifdef WIRELESS_EXT
    unsigned short iobase = dev->base_addr;
    netwavep_private *priv = (netwavep_private *) dev->priv;
    u_char *ramBase = priv->ramBase;
    struct iwreq *wrq = (struct iwreq *) rq;
#endif
	
    DEBUG( 0, "%s: ->netwavep_ioctl(cmd=0x%X)\n", dev->name, cmd);
	
    /* Disable interrupts & save flags */
    save_flags(flags);
    cli();

    /* Look what is the request */
    switch(cmd) {
	/* --------------- WIRELESS EXTENSIONS --------------- */
#ifdef WIRELESS_EXT
    case SIOCGIWNAME:
	/* Get name */
	strcpy(wrq->u.name, "Netwave");
	break;
    case SIOCSIWNWID:
	/* Set domain */
	if(wrq->u.nwid.on) {
	    domain = wrq->u.nwid.nwid;
	    printk( KERN_DEBUG "Setting domain to 0x%x%02x\n", 
		    (domain >> 8) & 0x01, domain & 0xff);
	    wait_WOC(iobase);
	    //writeb(NETWAVEP_CMD_SMD, ramBase + NETWAVEP_EREG_CB + 0);
	    //writeb( domain & 0xff, ramBase + NETWAVEP_EREG_CB + 1);
	    //writeb((domain >>8 ) & 0x01,ramBase + NETWAVEP_EREG_CB+2);
	    //writeb(NETWAVEP_CMD_EOC, ramBase + NETWAVEP_EREG_CB + 3);
	} break;
    case SIOCGIWNWID:
	/* Read domain*/
	wrq->u.nwid.nwid = domain;
	wrq->u.nwid.on = 1;
	break;
    case SIOCGIWENCODE:
	/* Get scramble key */
	wrq->u.encoding.code = scramble_key;
	wrq->u.encoding.method = 1;
	break;
    case SIOCSIWENCODE:
	/* Set  scramble key */
	scramble_key = wrq->u.encoding.code;
	wait_WOC(iobase);
	//writeb(NETWAVEP_CMD_SSK, ramBase + NETWAVEP_EREG_CB + 0);
	//writeb(scramble_key & 0xff, ramBase + NETWAVEP_EREG_CB + 1);
	//writeb((scramble_key>>8) & 0xff, ramBase + NETWAVEP_EREG_CB + 2);
	//writeb(NETWAVEP_CMD_EOC, ramBase + NETWAVEP_EREG_CB + 3);
	break;
   case SIOCGIWRANGE:
       /* Basic checking... */
       if(wrq->u.data.pointer != (caddr_t) 0) {
	   struct iw_range	range;
		   
	   /* Verify the user buffer */
	   ret = verify_area(VERIFY_WRITE, wrq->u.data.pointer,
			     sizeof(struct iw_range));
	   if(ret)
	       break;
		   
	   /* Set the length (useless : its constant...) */
	   wrq->u.data.length = sizeof(struct iw_range);
		   
	   /* Set information in the range struct */
	   range.throughput = 1.6 * 1024 * 1024;	/* don't argue on this ! */
	   range.min_nwid = 0x0000;
	   range.max_nwid = 0x01FF;
		   
	   range.num_channels = range.num_frequency = 0;
		   
	   range.sensitivity = 0x3F;
	   range.max_qual.qual = 255;
	   range.max_qual.level = 255;
	   range.max_qual.noise = 0;
		   
	   /* Copy structure to the user buffer */
	   copy_to_user(wrq->u.data.pointer, &range,
			sizeof(struct iw_range));
       }
       break;
    case SIOCGIWPRIV:
	/* Basic checking... */
	if(wrq->u.data.pointer != (caddr_t) 0) {
	    struct iw_priv_args	priv[] =
	    {	/* cmd,		set_args,	get_args,	name */
		{ SIOCGIPSNAP, IW_PRIV_TYPE_BYTE | IW_PRIV_SIZE_FIXED | 0, 
		  sizeof(struct site_survey), 
		  "getsitesurvey" },
	    };
			
	    /* Verify the user buffer */
	    ret = verify_area(VERIFY_WRITE, wrq->u.data.pointer,
			      sizeof(priv));
	    if(ret)
		break;
	    
	    /* Set the number of ioctl available */
	    wrq->u.data.length = 1;
			
	    /* Copy structure to the user buffer */
	    copy_to_user(wrq->u.data.pointer, (u_char *) priv,
			 sizeof(priv));
	} 
	break;
    case SIOCGIPSNAP:
	if(wrq->u.data.pointer != (caddr_t) 0) {
	    /* Take snapshot of environment */
	    netwavep_snapshot( priv, ramBase, iobase);
	    /* Verify the user buffer */
	    ret = verify_area(VERIFY_WRITE, wrq->u.data.pointer,
			      sizeof(struct site_survey));
	    if(ret) {
		printk(KERN_DEBUG "Bad buffer!\n");
		break;
	    }
	    wrq->u.data.length = priv->nss.length;
	    /* Copy structure to the user buffer */
	    copy_to_user(wrq->u.data.pointer, 
			 (u_char *) &priv->nss,
			 sizeof( struct site_survey));

	    priv->lastExec = jiffies;
	}
	break;
#endif
    default:
	ret = -EOPNOTSUPP;
    }
	
    /* ReEnable interrupts & restore flags */
    restore_flags(flags);
    
    return ret;
}

/*
 * Function netwavep_pcmcia_config (link)
 *
 *     netwavep_pcmcia_config() is scheduled to run after a CARD_INSERTION 
 *     event is received, to configure the PCMCIA socket, and to make the
 *     device available to the system. 
 *
 */

#define CS_CHECK(fn, args...) \
while ((last_ret=CardServices(last_fn=(fn), args))!=0) goto cs_failed

static void netwavep_pcmcia_config(dev_link_t *link) {
    client_handle_t handle;
    tuple_t tuple;
    cisparse_t parse;
    struct device *dev;
    int i, j, last_ret, last_fn;
    u_char buf[64];
    win_req_t req;
    memreq_t mem;
    u_char *ramBase = NULL;
    /*    modwin_t mod;
	  short iobase, *phys_addr;
	  */  
    handle = link->handle;
    dev = link->priv;

    DEBUG(0, "netwavep_pcmcia_config(0x%p)\n", link);

    /*
      This reads the card's CONFIG tuple to find its configuration
      registers.
    */
    tuple.Attributes = 0;
    tuple.TupleData = (cisdata_t *) buf;
    tuple.TupleDataMax = 64;
    tuple.TupleOffset = 0;
    tuple.DesiredTuple = CISTPL_CONFIG;
    CS_CHECK(GetFirstTuple, handle, &tuple);
    CS_CHECK(GetTupleData, handle, &tuple);
    CS_CHECK(ParseTuple, handle, &tuple, &parse);
    link->conf.ConfigBase = parse.config.base;
    link->conf.Present = parse.config.rmask[0];
    
    /* Configure card */
    link->state |= DEV_CONFIG;
	
    /*
     *  Try allocating IO ports.  This tries a few fixed addresses.
     *  If you want, you can also read the card's config table to
     *  pick addresses -- see the serial driver for an example.
     */
    for (j = 0x0; j < 0x400; j += 0x20) {
	link->io.BasePort1 = j ^ 0x300;
	i = CardServices(RequestIO, link->handle, &link->io);
	if (i == CS_SUCCESS) break;
    }
    if (i != CS_SUCCESS) {
	cs_error(link->handle, RequestIO, i);
	goto failed;
    }
		
    /*
     *  Now allocate an interrupt line.  Note that this does not
     *  actually assign a handler to the interrupt.
     */
    CS_CHECK(RequestIRQ, handle, &link->irq);
	
    /*
     *  This actually configures the PCMCIA socket -- setting up
     *  the I/O windows and the interrupt mapping.
     */
    CS_CHECK(RequestConfiguration, handle, &link->conf);
    
    /*
     *  Allocate a 32K memory window.  Note that the dev_link_t
     *  structure provides space for one window handle -- if your
     *  device needs several windows, you'll need to keep track of
     *  the handles in your private data structure, link->priv.
     */
    DEBUG(1, "Setting mem speed of %d\n", mem_speed);
    
    req.Attributes = WIN_DATA_WIDTH_8|WIN_MEMORY_TYPE_CM|WIN_ENABLE;
    req.Base = 0; req.Size = 0x8000;
    req.AccessSpeed = mem_speed;
    link->win = (window_handle_t)link->handle;
    CS_CHECK(RequestWindow, &link->win, &req);
    mem.CardOffset = 0x20000; mem.Page = 0; 
    CS_CHECK(MapMemPage, link->win, &mem);
    
    /* Store base address of the common window frame */
    ramBase = ioremap(req.Base, 0x8000);
    ((netwavep_private*)dev->priv)->ramBase = ramBase;
    
    dev->irq = link->irq.AssignedIRQ;
    dev->base_addr = link->io.BasePort1;
    dev->tbusy = 0;
    if (register_netdev(dev) != 0) {
	printk(KERN_DEBUG "netwavep_cs: register_netdev() failed\n");
	goto failed;
    }
	
    link->state &= ~DEV_CONFIG_PENDING;
	
    link->dev = &((netwavep_private *)dev->priv)->node;

    /* Reset card before reading physical address */
    netwavep_doreset(dev->base_addr, ramBase);
    
    /* Read the ethernet address and fill in the Netwave registers. */
    //    for (i = 0; i < 6; i++) 
    //	dev->dev_addr[i] = readb(ramBase + NETWAVEP_EREG_PA + i);

    //    printk(KERN_INFO "%s: Netwave: port %#3lx, irq %d, mem %lx id "
    //	   "%c%c, hw_addr ", dev->name, dev->base_addr, dev->irq,
    //	   (u_long) ramBase, (int) readb(ramBase+NETWAVEP_EREG_NI),
    //	   (int) readb(ramBase+NETWAVEP_EREG_NI+1));
    for (i = 0; i < 6; i++)
	printk("%02X%s", dev->dev_addr[i], ((i<5) ? ":" : "\n"));
    {
	char ost[40];

	for (i = 0; i < 32; i++)
	    ost[i] = readb(ramBase + NWP_BANN + i);
	ost[32] = 0;
	printk(KERN_INFO "Netwave Airsurfer Plus version: %s\n", 
	       ost);
    }

    /* get revision words */
    printk(KERN_DEBUG "Netwavep_reset: revision %04x %04x\n", 
	   get_uint16(ramBase + NETWAVEP_EREG_ARW),
	   get_uint16(ramBase + NETWAVEP_EREG_ARW+2));
    return;

cs_failed:
    cs_error(link->handle, last_fn, last_ret);
failed:
    netwavep_release((u_long)link);
    return;
} /* netwavep_pcmcia_config */

/*
 * Function netwavep_release (arg)
 *
 *    After a card is removed, netwavep_release() will unregister the net
 *    device, and release the PCMCIA configuration.  If the device is
 *    still open, this will be postponed until it is closed.
 */
static void netwavep_release(u_long arg) {
    dev_link_t *link = (dev_link_t *)arg;
    struct device *dev = link->priv;

    DEBUG(0, "netwavep_release(0x%p)\n", link);

    /*
      If the device is currently in use, we won't release until it
      is actually closed.
      */
    if (link->open) {
	printk(KERN_DEBUG "netwavep_cs: release postponed, '%s' still open\n",
	       link->dev->dev_name);
	link->state |= DEV_STALE_CONFIG;
	return;
    }
	
    if (link->dev != '\0')
	unregister_netdev(dev);
    
    /* Unlink the device chain */
    link->dev = NULL;
    
    /* Don't bother checking to see if these succeed or not */
    if (link->win) {
	iounmap(((netwavep_private *)dev->priv)->ramBase);
	CardServices(ReleaseWindow, link->win);
    }
    CardServices(ReleaseConfiguration, link->handle);
    CardServices(ReleaseIO, link->handle, &link->io);
    CardServices(ReleaseIRQ, link->handle, &link->irq);
 
    link->state &= ~DEV_CONFIG;
    
    if (link->state & DEV_STALE_LINK)
	netwavep_detach(link);
    
} /* netwavep_release */

/*
 * Function netwavep_event (event, priority, args)
 *
 *    The card status event handler.  Mostly, this schedules other
 *    stuff to run after an event is received.  A CARD_REMOVAL event
 *    also sets some flags to discourage the net drivers from trying
 *    to talk to the card any more.
 *
 *    When a CARD_REMOVAL event is received, we immediately set a flag
 *    to block future accesses to this device.  All the functions that
 *    actually access the device should check this flag to make sure
 *    the card is still present.
 *
 */
static int netwavep_event(event_t event, int priority,
			 event_callback_args_t *args) {
    dev_link_t *link = args->client_data;
    struct device *dev = link->priv;
	
    DEBUG(1, "netwavep_event(0x%06x)\n", event);
  
    switch (event) {
    case CS_EVENT_REGISTRATION_COMPLETE:
	DEBUG(0, "netwavep_cs: registration complete\n");
	break;

    case CS_EVENT_CARD_REMOVAL:
	link->state &= ~DEV_PRESENT;
	if (link->state & DEV_CONFIG) {
	    dev->tbusy = 1; dev->start = 0;
	    /* ((netwavep_private *)link->priv)->block = 1; */
	    link->release.expires = RUN_AT(5);
	    add_timer(&link->release);
	}
	break;
    case CS_EVENT_CARD_INSERTION:
	link->state |= DEV_PRESENT | DEV_CONFIG_PENDING;
	netwavep_pcmcia_config( link);
	break;
    case CS_EVENT_PM_SUSPEND:
	link->state |= DEV_SUSPEND;
	/* Fall through... */
    case CS_EVENT_RESET_PHYSICAL:
	if (link->state & DEV_CONFIG) {
	    if (link->open) {
		dev->tbusy = 1; dev->start = 0;
	    }
	    CardServices(ReleaseConfiguration, link->handle);
	}
	break;
    case CS_EVENT_PM_RESUME:
	link->state &= ~DEV_SUSPEND;
	/* Fall through... */
    case CS_EVENT_CARD_RESET:
	if (link->state & DEV_CONFIG) {
	    CardServices(RequestConfiguration, link->handle, &link->conf);
	    if (link->open) {
		netwavep_reset(dev);
		dev->tbusy = 0; dev->start = 1;
	    }
	}
	break;
    }
    return 0;
} /* netwavep_event */

/*
 * Function netwavep_doreset (ioBase, ramBase)
 *
 *    Proper hardware reset of the card.
 */
static void netwavep_doreset(unsigned long ioBase, u_char* ramBase) {
    /* Reset card */
    //    wait_WOC(ioBase);
    //    outb(0x80, ioBase + NETWAVEP_REG_PMR);
    //    writeb(0x08, ramBase + NETWAVEP_EREG_ASCC); /* Bit 3 is WOC */
    //    outb(0x0, ioBase + NETWAVEP_REG_PMR); /* release reset */
}

/*
 * Function netwavep_reset (dev)
 *
 *    Reset and restore all of the netwave registers 
 */
static void netwavep_reset(struct device *dev) {
    /* u_char state; */
    netwavep_private *priv = (netwavep_private*) dev->priv;
    u_char *ramBase = priv->ramBase;
    unsigned long iobase = dev->base_addr;
    char ost[40];
    NWP_ConStat_Block t;
    //NWP_C_Block       c;
    int i;
    u_char CRes;
    NWP_Init_Tx_Type itt;
    NWP_Enable_Receiver_Type ers;

    DEBUG(0, "netwavep_reset: Done with hardware reset\n");

    priv->timeoutCounter = 0;
	
    /* If watchdog was activated, kill it ! */
    del_timer(&priv->watchdog);
	

    /* Check banner */

    for (i = 0; i < 32; i++)
	ost[i] = readb(ramBase + NWP_BANN + i);
    ost[32] = 0;
    printk(KERN_INFO "Netwave Airsurfer Plus version: %s\n", 
	       ost);

    /* Check status etc */
    copy_from_pc(&t, ramBase + NWP_CSB, sizeof(t));
    printk(KERN_INFO "NWP: (%x %x %x %x) (%x %x %x %x) (%x %x %x)\n", 
	   t.Self_Test_Status,
	   t.STA_State, 
	   t.Rsvd_For_User_Rtn, 
	   t.Interrupt_status, 
	   
	   t.Interrupt_Mask,
	   t.Lockout_PCnet_MOBILE, 
	   t.Lockout_Host,
	   t.Interrupt_Status2,
	   
	   t.SW_Dis_Pwr_Dn, 
	   t.Interrupt_Mask2,
	   t.Driver_State);

    
    /* Write a test command */
    writeb(NWP_CMD_TEST, ramBase + NWP_CMD + 0); 
    writeb(0, ramBase + NWP_CMD + 1); 
    CRes = wait_CStatus(ramBase);

    printk(KERN_INFO "NWP: Test command result: %d\n", CRes);
    writeb(0, ramBase + NWP_CMD);

    /* Get Local_MIB 
     * 4 is offset to params,  
     */
    writeb(0,    ramBase + NWP_CMD + 4);                  // Type = Local_MIB
    writeb(sizeof(local_mib), ramBase + NWP_CMD + 5); // Size
    writeb(0,    ramBase + NWP_CMD + 6);                  // Index

    // Start command
    writeb(NWP_CMD_GMIB, ramBase + NWP_CMD + 0);          
    writeb(0, ramBase + NWP_CMD + 1); 
    
    CRes = wait_CStatus(ramBase);
    
    /* Now copy in struct data */
    copy_from_pc(&local_mib, ramBase + NWP_CMD + 4 + 4, sizeof(local_mib));

    //		 sizeof(NWP_LMIB_Type));
    printk(KERN_INFO "NWP: Reading %d bytes from %x\n", 
	   sizeof(local_mib), NWP_CMD+4+4);
    for (i = 0; i < sizeof(local_mib); i++)
	((u_char*) &local_mib)[i] = readb(ramBase + NWP_CMD + 4 + 4 + i);

    printk(KERN_INFO "NWP: local-mib. AP %d, mode %d - off %x %x %x %x \n", 
	   local_mib.Acting_as_AP, 
	   local_mib.Network_Mode, 
	   (unsigned int) local_mib.Tx_Buffer_Offset,
	   (unsigned int) local_mib.Tx_Buffer_Size,
	   (unsigned int) local_mib.Rx_Buffer_Offset,
	   (unsigned int) local_mib.Rx_Buffer_Size);

    // Now modify the cards settings
    // I'm not an access point
    local_mib.Network_Mode = 1; // infrastructure lan, 0 = ad_hoc
    local_mib.Acting_as_AP = 0;



    // Write back entire struct
    copy_to_pc(ramBase + NWP_CMD + 4 + 4, &local_mib, sizeof(local_mib));
    writeb(0,                 ramBase + NWP_CMD + 4); // Type = Local_MIB
    writeb(sizeof(local_mib), ramBase + NWP_CMD + 5); // Size
    writeb(0,                 ramBase + NWP_CMD + 6); // Index
    // Start command
    writeb(NWP_CMD_SMIB, ramBase + NWP_CMD + 0);          
    writeb(0,            ramBase + NWP_CMD + 1); 
    
    CRes = wait_CStatus(ramBase);
    printk(KERN_INFO "NWP: Set Local Mib result %d\n", CRes);

    // enable tx
    itt.Data_Desc = local_mib.Tx_Buffer_Offset;
    itt.Mgmt_Desc = 0;	
    itt.Broadcast_Desc = 0;
    itt.PS_Poll_Desc = 0;
    itt.CF_Poll_Desc = 0;
    copy_to_pc(ramBase + NWP_CMD + 4, &itt, sizeof(itt));
    // Start command
    writeb(NWP_CMD_INITX, ramBase + NWP_CMD + 0);          
    writeb(0,             ramBase + NWP_CMD + 1); 
    
    CRes = wait_CStatus(ramBase);
    printk(KERN_INFO "NWP: Init Tx result %d\n", CRes);


    // enable rx
    
    writeb(NWP_CMD_ENRX, ramBase + NWP_CMD + 0);          
    writeb(0,            ramBase + NWP_CMD + 1); 
    
    CRes = wait_CStatus(ramBase);

    copy_from_pc(&ers, ramBase + NWP_CMD + 4, sizeof(ers));
    printk(KERN_INFO "NWP: Enable Receiver result %d, DD %x, PPMD %x\n", CRes, 
	   (unsigned int) ers.Data_Desc, 
	   (unsigned int) ers.PS_Poll_Mgmt_Desc);




    // Initiate a SCAN
    writeb(NWP_CMD_SCAN, ramBase + NWP_CMD + 0);          
    writeb(0,            ramBase + NWP_CMD + 1); 
    CRes = wait_CStatus(ramBase);



    /* Check status etc */
    copy_from_pc(&t, ramBase + NWP_CSB, sizeof(t));
    printk(KERN_INFO "NWP: (%x %x %x %x) (%x %x %x %x) (%x %x %x)\n", 
	   t.Self_Test_Status,
	   t.STA_State, 
	   t.Rsvd_For_User_Rtn, 
	   t.Interrupt_status, 
	   
	   t.Interrupt_Mask,
	   t.Lockout_PCnet_MOBILE, 
	   t.Lockout_Host,
	   t.Interrupt_Status2,
	   
	   t.SW_Dis_Pwr_Dn, 
	   t.Interrupt_Mask2,
	   t.Driver_State);







    
    /* Reset card */
    netwavep_doreset(iobase, ramBase);
    printk(KERN_DEBUG "netwavep_reset: Done with hardware reset\n");
	
    /* Write a NOP to check the card */
    wait_WOC(iobase);
    //writeb(NETWAVEP_CMD_NOP, ramBase + NETWAVEP_EREG_CB + 0);
    //writeb(NETWAVEP_CMD_EOC, ramBase + NETWAVEP_EREG_CB + 1);
	
    /* Set receive conf */
    wait_WOC(iobase);
    //writeb(NETWAVEP_CMD_SRC, ramBase + NETWAVEP_EREG_CB + 0);
    //writeb(rxConfRxEna + rxConfBcast, ramBase + NETWAVEP_EREG_CB + 1);
    //writeb(NETWAVEP_CMD_EOC, ramBase + NETWAVEP_EREG_CB + 2);
    
    /* Set transmit conf */
    wait_WOC(iobase);
    //writeb(NETWAVEP_CMD_STC, ramBase + NETWAVEP_EREG_CB + 0);
    //writeb(txConfTxEna, ramBase + NETWAVEP_EREG_CB + 1);
    //writeb(NETWAVEP_CMD_EOC, ramBase + NETWAVEP_EREG_CB + 2);
    
    /* Now set the MU Domain */
    printk(KERN_DEBUG "Setting domain to 0x%x%02x\n", (domain >> 8) & 0x01, domain & 0xff);
    wait_WOC(iobase);
    //writeb(NETWAVEP_CMD_SMD, ramBase + NETWAVEP_EREG_CB + 0);
    //writeb(domain & 0xff, ramBase + NETWAVEP_EREG_CB + 1);
    //writeb((domain>>8) & 0x01, ramBase + NETWAVEP_EREG_CB + 2);
    //writeb(NETWAVEP_CMD_EOC, ramBase + NETWAVEP_EREG_CB + 3);
	
    /* Set scramble key */
    printk(KERN_DEBUG "Setting scramble key to 0x%x\n", scramble_key);
    wait_WOC(iobase);
    //writeb(NETWAVEP_CMD_SSK, ramBase + NETWAVEP_EREG_CB + 0);
    //writeb(scramble_key & 0xff, ramBase + NETWAVEP_EREG_CB + 1);
    //writeb((scramble_key>>8) & 0xff, ramBase + NETWAVEP_EREG_CB + 2);
    //writeb(NETWAVEP_CMD_EOC, ramBase + NETWAVEP_EREG_CB + 3);

    /* Enable interrupts, bit 4 high to keep unused
     * source from interrupting us, bit 2 high to 
     * set interrupt enable, 567 to enable TxDN, 
     * RxErr and RxRdy
     */
    wait_WOC(iobase);
    //outb(imrConfIENA+imrConfRFU1, iobase + NETWAVEP_REG_IMR);

    /* Hent 4 bytes fra 0x170. Skal vaere 0a,29,88,36
     * waitWOC
     * skriv 80 til d000:3688
     * sjekk om det ble 80
     */
    
    /* Enable Receiver */
    wait_WOC(iobase);
    //writeb(NETWAVEP_CMD_ER, ramBase + NETWAVEP_EREG_CB + 0);
    //writeb(NETWAVEP_CMD_EOC, ramBase + NETWAVEP_EREG_CB + 1);
	
    /* Set the IENA bit in COR */
    wait_WOC(iobase);
    //outb(corConfIENA + corConfLVLREQ, iobase + NETWAVEP_REG_COR);
}

/*
 * Function netwavep_config (dev, map)
 *
 *    Configure device, this work is done by netwavep_pcmcia_config when a
 *    card is inserted
 */
static int netwavep_config(struct device *dev, struct ifmap *map) {
    return 0; 
}

/*
 * Function netwavep_hw_xmit (data, len, dev)    
 */
static int netwavep_hw_xmit(unsigned char* data, int len, struct device* dev) {
    unsigned long flags;
    unsigned int TxFreeList,
	         curBuff,
	         MaxData, 
                 DataOffset;
    int tmpcount; 
	
    netwavep_private *priv = (netwavep_private *) dev->priv;
    u_char* ramBase = priv->ramBase;
    unsigned long iobase = dev->base_addr;
	
	
    /* Disable interrupts & save flags */
    save_flags(flags);
    cli();

    /* Check if there are transmit buffers available */
    wait_WOC(iobase);
    if ((inb(iobase+NETWAVEP_REG_ASR) & NETWAVEP_ASR_TXBA) == 0) {
	/* No buffers available */
	printk(KERN_DEBUG "netwavep_hw_xmit: %s - no xmit buffers available.\n",
	       dev->name);
	return 1;
    }
	
#if (LINUX_VERSION_CODE >= VERSION(2,1,25))
    priv->stats.tx_bytes += len;
#endif

    DEBUG(3, "Transmitting with SPCQ %x SPU %x LIF %x ISPLQ %x\n",
	  readb(ramBase + NETWAVEP_EREG_SPCQ),
	  readb(ramBase + NETWAVEP_EREG_SPU),
	  readb(ramBase + NETWAVEP_EREG_LIF),
	  readb(ramBase + NETWAVEP_EREG_ISPLQ));

    /* Now try to insert it into the adapters free memory */
    wait_WOC(iobase);
    TxFreeList = get_uint16(ramBase + NETWAVEP_EREG_TDP);
    MaxData    = get_uint16(ramBase + NETWAVEP_EREG_TDP+2);
    DataOffset = get_uint16(ramBase + NETWAVEP_EREG_TDP+4);
	
    DEBUG(3, "TxFreeList %x, MaxData %x, DataOffset %x\n",
	  TxFreeList, MaxData, DataOffset);

    /* Copy packet to the adapter fragment buffers */
    curBuff = TxFreeList; 
    tmpcount = 0; 
    while (tmpcount < len) {
	int tmplen = len - tmpcount; 
	copy_to_pc(ramBase + curBuff + DataOffset, data + tmpcount, 
		   (tmplen < MaxData) ? tmplen : MaxData);
	tmpcount += MaxData;
			
	/* Advance to next buffer */
	curBuff = get_uint16(ramBase + curBuff);
    }
    
    /* Now issue transmit list */
    wait_WOC(iobase);
    //writeb(NETWAVEP_CMD_TL, ramBase + NETWAVEP_EREG_CB + 0);
    //writeb(len & 0xff, ramBase + NETWAVEP_EREG_CB + 1);
    //writeb((len>>8) & 0xff, ramBase + NETWAVEP_EREG_CB + 2);
    //writeb(NETWAVEP_CMD_EOC, ramBase + NETWAVEP_EREG_CB + 3);
	
    /* If watchdog not already active, activate it... */
    if(priv->watchdog.prev == (struct timer_list *) NULL) {

	/* set timer to expire in WATCHDOG_JIFFIES */
	priv->watchdog.expires = jiffies + WATCHDOG_JIFFIES;
	add_timer(&priv->watchdog);
    }
    restore_flags( flags);
    return 0;
}

static int netwavep_start_xmit(struct sk_buff *skb, struct device *dev) {
	/* This flag indicate that the hardware can't perform a transmission.
	 * Theoritically, NET3 check it before sending a packet to the driver,
	 * but in fact it never do that and pool continuously.
	 * As the watchdog will abort too long transmissions, we are quite safe...
	 */

    if (dev->tbusy) {
	/* Handled by watchdog */
	return 1;
		
	/* If we get here, some higher level has decided we are broken.
	   There should really be a 'kick me' function call instead.
	   */
	/*int tickssofar = jiffies - dev->trans_start;*/
	/* printk("xmit called with busy. tickssofar %d\n", tickssofar); */
	/*if (tickssofar < TX_TIMEOUT) 
	  return 1;
	  */
	/* Should also detect if the kernel tries to xmit
	 * on a stopped card. 
	 */
       
	/*if (netwavep_debug > 0)
	  printk(KERN_DEBUG "%s timed out.\n", dev->name);
	  netwavep_reset(dev); 
	  dev->trans_start = jiffies;
	  dev->tbusy = 0;*/
    }

    /* Sending a NULL skb means some higher layer thinks we've missed an
     * tx-done interrupt. Caution: dev_tint() handles the cli()/sti()
     * itself. 
     */

#if (LINUX_VERSION_CODE < VERSION(2,1,25))
    if (skb == NULL) {
        dev_tint(dev);
	return 0;
    }
   
    if (skb->len <= 0)
	return 0;
#endif

    /* Block a timer-based transmit from overlapping. This could 
     * better be done with atomic_swap(1, dev->tbusy, but set_bit()
     * works as well 
     */
    if ( test_and_set_bit(0, (void*)&dev->tbusy) != 0) 
	printk("%s: Transmitter access conflict.\n", dev->name);
    else {
	short length = ETH_ZLEN < skb->len ? skb->len : ETH_ZLEN;
	unsigned char* buf = skb->data;
	
	if (netwavep_hw_xmit( buf, length, dev) == 1) {
	    /* Some error, let's make them call us another time? */
	    dev->tbusy = 0;
	}
	dev->trans_start = jiffies;
    }
    DEV_KFREE_SKB(skb);
    
    return 0;
} /* netwavep_start_xmit */

/*
 * Function netwavep_interrupt IRQ (irq, dev_id, regs)
 *
 *    This function is the interrupt handler for the Netwave card. This
 *    routine will be called whenever: 
 *	  1. A packet is received.
 *	  2. A packet has successfully been transfered and the unit is
 *	     ready to transmit another packet.
 *	  3. A command has completed execution.
 */
static void netwavep_interrupt IRQ(int irq, void* dev_id, struct pt_regs *regs) {
    unsigned long iobase;
    u_char *ramBase;
    struct device *dev = (struct device *)DEV_ID;
    struct netwavep_private *priv;
    int i;
    dev_link_t *link;
    
	if ((dev == NULL) | (!dev->start))
		return;
    
    priv = (netwavep_private *)dev->priv;
    
    if (dev->interrupt) {
	printk("%s: re-entering the interrupt handler.\n", dev->name);
	return;
    }
    dev->interrupt = 1;
	
    /* Find the correct dev_link_t */
    for (link = dev_list; NULL != link; link = link->next)
	if (dev == link->priv) break;
    
    iobase = dev->base_addr;
    ramBase = priv->ramBase;
	
    /* Now find what caused the interrupt, check while interrupts ready */
    for (i = 0; i < 10; i++) {
	u_char status;
		
	wait_WOC(iobase);	
	if (!(inb(iobase+NETWAVEP_REG_CCSR) & 0x02))
	    break; /* None of the interrupt sources asserted */
	
        status = inb(iobase + NETWAVEP_REG_ASR);
		
	if ( ! (link->state & DEV_PRESENT) || link->state & DEV_SUSPEND ) {
	    DEBUG( 1, "netwavep_interupt: Interrupt with status 0x%x "
		   "from removed or suspended card!\n", status);
	    break;
	}
		
	/* RxRdy */
	if (status & 0x80) {
	    netwavep_rx(dev);
	    /* wait_WOC(iobase); */
	    /* RxRdy cannot be reset directly by the host */
	}
	/* RxErr */
	if (status & 0x40) {
	    u_char rser;
			
	    rser = readb(ramBase + NETWAVEP_EREG_RSER);			
	    
	    if (rser & 0x04) {
		++priv->stats.rx_dropped; 
		++priv->stats.rx_crc_errors;
	    }
	    if (rser & 0x02)
		++priv->stats.rx_frame_errors;
			
	    /* Clear the RxErr bit in RSER. RSER+4 is the
	     * write part. Also clear the RxCRC (0x04) and 
	     * RxBig (0x02) bits if present */
	    wait_WOC(iobase);
	    //writeb(0x40 | (rser & 0x06), ramBase + NETWAVEP_EREG_RSER + 4);

	    /* Write bit 6 high to ASCC to clear RxErr in ASR,
	     * WOC must be set first! 
	     */
	    wait_WOC(iobase);
	    //writeb(0x40, ramBase + NETWAVEP_EREG_ASCC);

	    /* Remember to count up priv->stats on error packets */
	    ++priv->stats.rx_errors;
	}
	/* TxDN */
	if (status & 0x20) {
	    int txStatus;

	    txStatus = readb(ramBase + NETWAVEP_EREG_TSER);
	    DEBUG(3, "Transmit done. TSER = %x id %x\n", 
		  txStatus, readb(ramBase + NETWAVEP_EREG_TSER + 1));
	    
	    if (txStatus & 0x20) {
		/* Transmitting was okay, clear bits */
		wait_WOC(iobase);
		//writeb(0x2f, ramBase + NETWAVEP_EREG_TSER + 4);
		++priv->stats.tx_packets;
	    }
			
	    if (txStatus & 0xd0) {
		if (txStatus & 0x80) {
		    ++priv->stats.collisions; /* Because of /proc/net/dev*/
		    /* ++priv->stats.tx_aborted_errors; */
		    /* printk("Collision. %ld\n", jiffies - dev->trans_start); */
		}
		if (txStatus & 0x40) 
		    ++priv->stats.tx_carrier_errors;
		/* 0x80 TxGU Transmit giveup - nine times and no luck
		 * 0x40 TxNOAP No access point. Discarded packet.
		 * 0x10 TxErr Transmit error. Always set when 
		 *      TxGU and TxNOAP is set. (Those are the only ones
		 *      to set TxErr).
		 */
		DEBUG(3, "netwavep_interrupt: TxDN with error status %x\n", 
		      txStatus);
		
		/* Clear out TxGU, TxNOAP, TxErr and TxTrys */
		wait_WOC(iobase);
		//writeb(0xdf & txStatus, ramBase+NETWAVEP_EREG_TSER+4);
		++priv->stats.tx_errors;
	    }
	    DEBUG(3, "New status is TSER %x ASR %x\n",
		  readb(ramBase + NETWAVEP_EREG_TSER),
		  inb(iobase + NETWAVEP_REG_ASR));
			
			
	    /* If watchdog was activated, kill it ! */
	    del_timer(&priv->watchdog);

	    dev->tbusy = 0;
	    mark_bh(NET_BH);
	}
	/* TxBA, this would trigger on all error packets received */
	/* if (status & 0x01) {
	   if (netwavep_debug > 3) 
	   printk(KERN_DEBUG "Transmit buffers available, %x\n", status); 
	   } 
	   */
    }
    /* done.. */
    dev->interrupt = 0;
    return;
} /* netwavep_interrupt */

/*
 * Function netwavep_watchdog (a)
 *
 *    Watchdog : when we start a transmission, we set a timer in the
 *    kernel.  If the transmission complete, this timer is disabled. If
 *    it expire, we reset the card.
 *
 */
static void netwavep_watchdog(u_long a) {
    struct device *dev;
    unsigned short iobase;
	
    dev = (struct device *) a;
    iobase = dev->base_addr;
    
    DEBUG( 1, "%s: netwavep_watchdog: watchdog timer expired\n", dev->name);
	
    netwavep_reset(dev); 
	
    /* We are not waiting anymore... */
    dev->tbusy = 0;
	
} /* netwavep_watchdog */

static struct enet_statistics *netwavep_get_stats(struct device *dev) {
    netwavep_private *priv = (netwavep_private*)dev->priv;

    update_stats(dev);

    DEBUG(2, "netwave: SPCQ %x SPU %x LIF %x ISPLQ %x MHS %x rxtx %x"
	  " %x tx %x %x %x %x\n", 
	  readb(priv->ramBase + NETWAVEP_EREG_SPCQ),
	  readb(priv->ramBase + NETWAVEP_EREG_SPU),
	  readb(priv->ramBase + NETWAVEP_EREG_LIF),
	  readb(priv->ramBase + NETWAVEP_EREG_ISPLQ),
	  readb(priv->ramBase + NETWAVEP_EREG_MHS),
	  readb(priv->ramBase + NETWAVEP_EREG_EC + 0xe),
	  readb(priv->ramBase + NETWAVEP_EREG_EC + 0xf),
	  readb(priv->ramBase + NETWAVEP_EREG_EC + 0x18),
	  readb(priv->ramBase + NETWAVEP_EREG_EC + 0x19),
	  readb(priv->ramBase + NETWAVEP_EREG_EC + 0x1a),
	  readb(priv->ramBase + NETWAVEP_EREG_EC + 0x1b));

    return &priv->stats;
}

static void update_stats(struct device *dev) {
    unsigned long flags;

    save_flags(flags);
    cli();

/*     netwavep_private *priv = (netwavep_private*) dev->priv;
    priv->stats.rx_packets = readb(priv->ramBase + 0x18e); 
    priv->stats.tx_packets = readb(priv->ramBase + 0x18f); */

    restore_flags(flags);
}

static int netwavep_rx(struct device *dev) {
    netwavep_private *priv = (netwavep_private*)(dev->priv);
    u_char *ramBase = priv->ramBase;
    u_long iobase   = dev->base_addr;
    u_char rxStatus;
    struct sk_buff *skb = NULL;
    unsigned int curBuffer,
		rcvList;
    int rcvLen;
    int tmpcount = 0;
    int dataCount, dataOffset;
    int i;
    u_char *ptr;
	
    DEBUG(3, "xinw_rx: Receiving ... \n");


    /* Receive max 10 packets for now. */
    for (i = 0; i < 10; i++) {
	/* Any packets? */
	wait_WOC(iobase);
	rxStatus = readb(ramBase + NETWAVEP_EREG_RSER);		
	if ( !( rxStatus & 0x80)) /* No more packets */
	    break;
		

	/* Check if multicast/broadcast or other */
	/* multicast = (rxStatus & 0x20);  */
		
	/* The receive list pointer and length of the packet */
	wait_WOC(iobase);
	rcvLen  = get_int16( ramBase + NETWAVEP_EREG_RDP);
	rcvList = get_uint16( ramBase + NETWAVEP_EREG_RDP + 2);
		
	if (rcvLen < 0) {
	    printk(KERN_DEBUG "netwavep_rx: Receive packet with len %d\n", 
		   rcvLen);
	    return 0;
	}
		
	/* Use the pcmcia_cs k_compat.h version */
	skb = ALLOC_SKB( rcvLen+3);		
	if (skb == NULL) {
	    DEBUG(1, "netwavep_rx: Could not allocate an sk_buff of "
		  "length %d\n", rcvLen);
	    ++priv->stats.rx_dropped; 
	    /* Tell the adapter to skip the packet */
	    wait_WOC(iobase);
	    //writeb(NETWAVEP_CMD_SRP, ramBase + NETWAVEP_EREG_CB + 0);
	    //writeb(NETWAVEP_CMD_EOC, ramBase + NETWAVEP_EREG_CB + 1);
	    return 0;
	}

#if (LINUX_VERSION_CODE >= VERSION(1,3,0))
	skb_reserve( skb, 2);  /* Align IP on 16 byte */
#endif
	skb_put( skb, rcvLen);
	skb->dev = dev;


	/* Copy packet fragments to the skb data area */
	ptr = (u_char*) skb->data;
	curBuffer = rcvList;
	tmpcount = 0; 
	while ( tmpcount < rcvLen) {
	    /* Get length and offset of current buffer */
	    dataCount  = get_uint16( ramBase+curBuffer+2);
	    dataOffset = get_uint16( ramBase+curBuffer+4);
		
	    copy_from_pc( ptr + tmpcount,
			  ramBase+curBuffer+dataOffset, dataCount);

	    tmpcount += dataCount;
		
	    /* Point to next buffer */
	    curBuffer = get_uint16(ramBase + curBuffer);
	}
	
#if (LINUX_VERSION_CODE >= VERSION(1,3,0)) 
	skb->protocol = eth_type_trans(skb,dev);
#endif
	/* Queue packet for network layer */
	netif_rx(skb);
		
	/* Got the packet, tell the adapter to skip it */
	wait_WOC(iobase);
	//writeb(NETWAVEP_CMD_SRP, ramBase + NETWAVEP_EREG_CB + 0);
	//writeb(NETWAVEP_CMD_EOC, ramBase + NETWAVEP_EREG_CB + 1);
	DEBUG(3, "Packet reception ok\n");
		
	priv->stats.rx_packets++;

#if (LINUX_VERSION_CODE >= VERSION(2,1,25))
	priv->stats.rx_bytes += skb->len;
#endif	
    }
    return 0;
}


static int netwavep_open(struct device *dev) {
    dev_link_t *link;

    DEBUG(2, "netwavep_open: starting.\n");

    for (link = dev_list; link; link = link->next)
	if (link->priv == dev) break;
    
    if (!DEV_OK(link))
	return -ENODEV;
	
    link->open++;
    MOD_INC_USE_COUNT;
	
    dev->interrupt = 0; dev->tbusy = 0; dev->start = 1;
    netwavep_reset(dev);
	
    return 0;
}

static int netwavep_close(struct device *dev) {
    dev_link_t *link;
    netwavep_private *priv = (netwavep_private *) dev->priv;
    
    for (link = dev_list; link; link = link->next)
	if (link->priv == dev) break;
    if (link == NULL)
	return -ENODEV;
	
    /* If watchdog was activated, kill it ! */
    del_timer(&priv->watchdog);
	
    link->open--;
    dev->start = 0;
    if (link->state & DEV_STALE_CONFIG) {
	link->release.expires = RUN_AT(5);
	link->state |= DEV_RELEASE_PENDING;
	add_timer(&link->release);
    }	
	
    MOD_DEC_USE_COUNT;
    return 0;
}

int init_module(void) {
    servinfo_t serv;

    DEBUG(0, "%s\n", version);

    CardServices(GetCardServicesInfo, &serv);
    if (serv.Revision != CS_RELEASE_CODE) {
	printk("netwavep_cs: Card Services release does not match!\n");
	return -1;
    }
 
    register_pcmcia_driver(&dev_info, &netwavep_attach, &netwavep_detach);
	
    return 0;
}

void cleanup_module(void) {
    DEBUG(1, "netwavep_cs: unloading\n");

    unregister_pcmcia_driver(&dev_info);
    while (dev_list != NULL) {
	if (dev_list->state & DEV_CONFIG)
	    netwavep_release((u_long)dev_list);
	netwavep_detach(dev_list);
    }
}


/* Set or clear the multicast filter for this adaptor.
   num_addrs == -1	Promiscuous mode, receive all packets
   num_addrs == 0	Normal mode, clear multicast list
   num_addrs > 0	Multicast mode, receive normal and MC packets, and do
   best-effort filtering.
 */
#ifdef NEW_MULTICAST
static void set_multicast_list(struct device *dev)
{
    short iobase = dev->base_addr;
    //u_char* ramBase = ((netwavep_private*) dev->priv)->ramBase;
    u_char  rcvMode = 0;
   
#ifdef PCMCIA_DEBUG
    if (pc_debug > 2) {
	static int old = 0;
	if (old != dev->mc_count) {
	    old = dev->mc_count;
	    DEBUG(0, "%s: setting Rx mode to %d addresses.\n",
		  dev->name, dev->mc_count);
	}
    }
#endif
	
    if (dev->mc_count || (dev->flags & IFF_ALLMULTI)) {
	/* Multicast Mode */
	rcvMode = rxConfRxEna + rxConfAMP + rxConfBcast;
    } else if (dev->flags & IFF_PROMISC) {
	/* Promiscous mode */
	rcvMode = rxConfRxEna + rxConfPro + rxConfAMP + rxConfBcast;
    } else {
	/* Normal mode */
	rcvMode = rxConfRxEna + rxConfBcast;
    }
	
    /* printk("netwave set_multicast_list: rcvMode to %x\n", rcvMode);*/
    /* Now set receive mode */
    wait_WOC(iobase);
    //writeb(NETWAVEP_CMD_SRC, ramBase + NETWAVEP_EREG_CB + 0);
    //writeb(rcvMode, ramBase + NETWAVEP_EREG_CB + 1);
    //writeb(NETWAVEP_CMD_EOC, ramBase + NETWAVEP_EREG_CB + 2);
}
#else
static void set_multicast_list(struct device *dev, int num_addrs, void *addrs) {
    short iobase = dev->base_addr;
    u_char* ramBase = ((netwavep_private*) dev->priv)->ramBase;
    u_char  rcvMode = 0;
	
    if (netwavep_debug > 3) {
	static int old = 0;
	if (old != num_addrs) {
	    old = num_addrs;
	    printk("%s: setting Rx mode to %d addresses.\n",
		   dev->name, num_addrs);
	}
    }
	
    if ((num_addrs > 0) || (num_addrs == -2)) {
	/* Multicast Mode */
	rcvMode = rxConfRxEna + rxConfAMP + rxConfBcast;
    } else if (num_addrs < 0) {
	/* Promiscous mode */
	rcvMode = rxConfRxEna + rxConfPro + rxConfAMP + rxConfBcast;
    } else {
	/* Normal mode */
	rcvMode = rxConfRxEna + rxConfBcast;
    }
	
    printk("netwave set_multicast_list: rcvMode to %x\n", rcvMode);
    /* Now set receive mode */
    wait_WOC(iobase);
    //writeb(NETWAVEP_CMD_SRC, ramBase + NETWAVEP_EREG_CB + 0);
    //writeb(rcvMode, ramBase + NETWAVEP_EREG_CB + 1);
    //writeb(NETWAVEP_CMD_EOC, ramBase + NETWAVEP_EREG_CB + 2);
}
#endif

