
/*

 */
 

#ifndef __RF24_CONFIG_H__
#define __RF24_CONFIG_H__

#define rf24_max(a,b) (a>b?a:b)
#define rf24_min(a,b) (a<b?a:b)


#define rf24_ss_pin			GPIO_C4
#define rf24_mosi_pin		GPIO_C5
#define rf24_miso_pin		GPIO_C6
#define rf24_sck_pin		GPIO_C7

#define rf24_ce_pin			GPIO_C1
#define rf24_csn_pin		GPIO_C2

#define rf24_dummy_pin		0
#define rf24_spi			SPIC


#endif // __RF24_CONFIG_H__

