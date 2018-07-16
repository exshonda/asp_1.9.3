/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2015 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 * 
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 * 
 *  @(#) $Id: target_serial.c 2708 2015-11-22 10:35:09Z ertl-honda $
 */

/*
 *  シリアルI/Oデバイス（SIO）ドライバ（SAML11用）
 */
#include <kernel.h>
#include <t_syslog.h>
#undef assert
#include "saml11.h"
#include "target_serial.h"
#include "hri_l11.h"
#include "peripheral_clk_config.h"
#include "hal_gpio.h"

/*
 *  シリアルI/Oポート初期化ブロックの定義
 */
typedef struct sio_port_initialization_block 
{
	Sercom*		p_sercom;		/* 使用するSERCOMへのポインタ */
	uint32_t	bps;			/* ボーレート */
}
SIOPINIB;

/*
 *  シリアルI/Oポート管理ブロックの定義
 */
struct sio_port_control_block 
{
	const SIOPINIB*	p_siopinib;	/* 初期化ブロック */
	intptr_t		exinf;		/* 拡張情報 */
	bool_t			openflag;	/* オープン済みフラグ */
	bool_t			devinitd;	/* デバイス初期化済みフラグ */
	bool_t			sendflag;	/* 送信割込みイネーブルフラグ */
	bool_t			getready;	/* 文字を受信した状態 */
	bool_t			putready;	/* 文字を送信できる状態 */
};

/*
 * シリアルI/Oポート初期化ブロック
 */
const SIOPINIB siopinib_table[TNUM_SIOP] = {
	{SERCOM2, 115200}
};

/*
 *  シリアルI/Oポート管理ブロックのエリア
 */
SIOPCB siopcb_table[TNUM_SIOP];

/*
 *  シリアルI/OポートIDから管理ブロックを取り出すためのマクロ
 */
#define INDEX_SIOP(siopid)  ((uint_t)((siopid) - 1))
#define get_siopcb(siopid)  (&(siopcb_table[INDEX_SIOP(siopid)]))

#define EDBG_COM_TX GPIO(GPIO_PORTA, 24)
#define EDBG_COM_RX GPIO(GPIO_PORTA, 25)

static void
sercom_uart_devinit(SIOPCB *p_siopcb){
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;

	/* デバイスが初期化済みならリターン */
	if (p_siopcb->devinitd) {
	  return;
	}

	p_siopcb->devinitd = true;

	/* Enable Clock and Power */
#if (defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U))
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM2_GCLK_ID_SLOW, CONF_GCLK_SERCOM2_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_mclk_set_APBCMASK_SERCOM2_bit(MCLK);
#else
#include "trustzone_veneer.h"
	nsc_periph_clock_init(SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC);
	nsc_periph_clock_init(SERCOM2_GCLK_ID_SLOW, CONF_GCLK_SERCOM2_SLOW_SRC);
#endif

	/* pin */
	gpio_set_pin_function(EDBG_COM_TX, PINMUX_PA24D_SERCOM2_PAD2);
	gpio_set_pin_function(EDBG_COM_RX, PINMUX_PA25D_SERCOM2_PAD3);

	/* Reset Uart */
	p_sercom->USART.CTRLA.bit.SWRST = 1 ;
	while(p_sercom->USART.CTRLA.bit.SWRST || p_sercom->USART.SYNCBUSY.bit.SWRST){};

	/* Set Baund Rate */
	p_sercom->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE(0x01) | SERCOM_USART_CTRLA_SAMPR(0);
	p_sercom->USART.BAUD.reg = 65536 - ((65536 * 16.0f * p_siopcb->p_siopinib->bps)/SERCOM_CLOCK_HZ);

	/* Disable All Interrupt */
	p_sercom->USART.INTENSET.reg = 0;

	/* Set Frame : 8bit, 1Stop, No parity, Lsb first*/
	p_sercom->USART.CTRLA.reg |= SERCOM_USART_CTRLA_FORM(0) |
								(1 << SERCOM_USART_CTRLA_DORD_Pos);
	p_sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_CHSIZE(0) |
								(0 << SERCOM_USART_CTRLB_SBMODE_Pos) |
								(0 << SERCOM_USART_CTRLB_PMODE_Pos);

	/* Set Pad */
	p_sercom->USART.CTRLA.reg |= SERCOM_USART_CTRLA_TXPO(0x01) | SERCOM_USART_CTRLA_RXPO(0x03);

	/* Tx/Rx Enable */
	p_sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN;

	/* Enable Uart */
	p_sercom->USART.CTRLA.bit.ENABLE = 0x01;
	while(p_sercom->USART.SYNCBUSY.bit.ENABLE){};
}

/*
 * カーネル起動時のログ出力用の初期化
 */
void
sercom_init_uart(uint32_t siopid)
{
	SIOPCB  *p_siopcb;

	p_siopcb = get_siopcb(siopid);
	p_siopcb->p_siopinib = &siopinib_table[INDEX_SIOP(siopid)];
	sercom_uart_devinit(p_siopcb);
}

/*
 *  UARTからのポーリング出力
 */
void
sercom_putc(uint32_t siopid, char c)
{
	Sercom* p_sercom = (get_siopcb(siopid))->p_siopinib->p_sercom;
	while(p_sercom->USART.INTFLAG.bit.DRE != SERCOM_USART_INTFLAG_DRE);
	p_sercom->USART.DATA.reg = (uint16_t)c;
}


/*
 * 文字を受信したか?
 */ 
Inline bool_t
uart_getready(SIOPCB *p_siopcb)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	return(p_sercom->USART.INTFLAG.bit.RXC);
}

/*
 * 文字を送信できるか?
 */
Inline bool_t
uart_putready(SIOPCB *p_siopcb)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	return(p_sercom->USART.INTFLAG.bit.DRE == SERCOM_USART_INTFLAG_DRE);
}

/*
 *  受信した文字の取り出し
 */
Inline uint8_t
uart_getchar(SIOPCB *p_siopcb)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	return p_sercom->USART.DATA.bit.DATA;
}

/*
 *  送信する文字の書き込み
 */
Inline void
uart_putchar(SIOPCB *p_siopcb, uint8_t c)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	p_sercom->USART.DATA.reg = (uint16_t)c;
}

/*
 *  送信割込み許可
 */
Inline void
uart_enable_send(SIOPCB *p_siopcb)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	p_sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_TXC;
}

/*
 *  送信割込み禁止
 */
Inline void
uart_disable_send(SIOPCB *p_siopcb)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	p_sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_TXC;
}


/*
 *  受信割込み許可
 */
Inline void
uart_enable_rcv(SIOPCB *p_siopcb)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	p_sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
}

/*
 *  受信割込み禁止
 */
Inline void
uart_disable_rcv(SIOPCB *p_siopcb)
{
	Sercom*	p_sercom = p_siopcb->p_siopinib->p_sercom;
	p_sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_RXC;
}


/*
 *  SIOドライバの初期化
 */
void
sio_initialize(intptr_t exinf)
{
	SIOPCB  *p_siopcb;
	uint_t  i;

	/*
	 *  シリアルI/Oポート管理ブロックの初期化
	 */
	for (p_siopcb = siopcb_table, i = 0; i < TNUM_SIOP; p_siopcb++, i++) {
		p_siopcb->p_siopinib = &(siopinib_table[i]);
		p_siopcb->openflag = false;
		p_siopcb->sendflag = false;
	}
}

/*
 * シリアルI/Oポートのオープン
 */
SIOPCB *
sercom_uart_opn_por(SIOPCB *p_siopcb, intptr_t exinf)
{
	p_siopcb->exinf = exinf;
	p_siopcb->getready = p_siopcb->putready = false;
	p_siopcb->openflag = true;

	sercom_uart_devinit(p_siopcb);

	return(p_siopcb);
}


/*
 *  シリアルI/Oポートのオープン
 */
SIOPCB *
sio_opn_por(ID siopid, intptr_t exinf)
{
	SIOPCB  *p_siopcb = get_siopcb(siopid);
	bool_t    opnflg;

	/*
	 *  オープンしたポートがあるかをopnflgに読んでおく．
	 */
	opnflg = p_siopcb->openflag;

	/*
	 *  デバイス依存のオープン処理．
	 */
	sercom_uart_opn_por(p_siopcb, exinf);

	/*
	 *  シリアルI/O割込みのマスクを解除する．
	 */
	if (!opnflg) {
		(void)ena_int(INTNO_SIO_RX);
		(void)ena_int(INTNO_SIO_TX);
	}
	return(p_siopcb);
}

/*
 *  シリアルI/Oポートのクローズ
 */
void
sio_cls_por(SIOPCB *p_siopcb)
{
	/*
	 *  デバイス依存のクローズ処理．
	 */
	p_siopcb->openflag = false;
    
	/*
	 *  シリアルI/O割込みをマスクする．
	 */
	dis_int(INTNO_SIO_RX);
	dis_int(INTNO_SIO_TX);
}

/*
 *  SIOの割込みハンドラ
 */
void
sio_isr(intptr_t exinf)
{
	SIOPCB *p_siopcb = get_siopcb(exinf);

	if (uart_getready(p_siopcb)) {
		/*
		 *  受信通知コールバックルーチンを呼び出す．
		 */
		sio_irdy_rcv(p_siopcb->exinf);
	}
	if (uart_putready(p_siopcb)) {
		/*
		 *  送信可能コールバックルーチンを呼び出す．
		 */
		sio_irdy_snd(p_siopcb->exinf);
	}	
}


/*
 *  シリアルI/Oポートへの文字送信
 */
bool_t
sio_snd_chr(SIOPCB *siopcb, char c)
{	
	if (uart_putready(siopcb)){
		uart_putchar(siopcb, c);
		return(true);
	}
	return(false);
}

/*
 *  シリアルI/Oポートからの文字受信
 */
int_t
sio_rcv_chr(SIOPCB *siopcb)
{
	if (uart_getready(siopcb)) {
		return((int_t)(uint8_t) uart_getchar(siopcb));
	}
	return(-1);
}

/*
 *  シリアルI/Oポートからのコールバックの許可
 */
void
sio_ena_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
	switch (cbrtn) {
		case SIO_RDY_SND:
			uart_enable_send(siopcb);
			break;
		case SIO_RDY_RCV:
			uart_enable_rcv(siopcb);
			break;
	}
}

/*
 *  シリアルI/Oポートからのコールバックの禁止
 */
void
sio_dis_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
	switch (cbrtn) {
		case SIO_RDY_SND:
			uart_disable_send(siopcb);
			break;
		case SIO_RDY_RCV:
			uart_disable_rcv(siopcb);
			break;
	}
}

#define UART_SERCOM   SERCOM2

void
serial_ena(void){
	UART_SERCOM->USART.INTENSET.reg |= SERCOM_USART_INTENSET_RXC;
}
	
void
serial_putc(char c){
	while(UART_SERCOM->USART.INTFLAG.bit.DRE != SERCOM_USART_INTFLAG_DRE);
	UART_SERCOM->USART.DATA.reg = (uint16_t)c;
}

char
serial_getc(void){
	while(!UART_SERCOM->USART.INTFLAG.bit.RXC);
	return UART_SERCOM->USART.DATA.bit.DATA;
}
