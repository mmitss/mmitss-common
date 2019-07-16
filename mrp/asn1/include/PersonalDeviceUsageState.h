//************************************************************************************************************
//
// © 2016-2019 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*************************************************************************************************************
/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../j2735_asn/J2735_201603DA.asn"
 * 	`asn1c -fcompound-names -gen-PER -gen-OER -pdu=auto`
 */

#ifndef	_PersonalDeviceUsageState_H_
#define	_PersonalDeviceUsageState_H_


#include <asn_application.h>

/* Including external dependencies */
#include <BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PersonalDeviceUsageState {
	PersonalDeviceUsageState_unavailable	= 0,
	PersonalDeviceUsageState_other	= 1,
	PersonalDeviceUsageState_idle	= 2,
	PersonalDeviceUsageState_listeningToAudio	= 3,
	PersonalDeviceUsageState_typing	= 4,
	PersonalDeviceUsageState_calling	= 5,
	PersonalDeviceUsageState_playingGames	= 6,
	PersonalDeviceUsageState_reading	= 7,
	PersonalDeviceUsageState_viewing	= 8
} e_PersonalDeviceUsageState;

/* PersonalDeviceUsageState */
typedef BIT_STRING_t	 PersonalDeviceUsageState_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_PersonalDeviceUsageState_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_PersonalDeviceUsageState;
asn_struct_free_f PersonalDeviceUsageState_free;
asn_struct_print_f PersonalDeviceUsageState_print;
asn_constr_check_f PersonalDeviceUsageState_constraint;
ber_type_decoder_f PersonalDeviceUsageState_decode_ber;
der_type_encoder_f PersonalDeviceUsageState_encode_der;
xer_type_decoder_f PersonalDeviceUsageState_decode_xer;
xer_type_encoder_f PersonalDeviceUsageState_encode_xer;
oer_type_decoder_f PersonalDeviceUsageState_decode_oer;
oer_type_encoder_f PersonalDeviceUsageState_encode_oer;
per_type_decoder_f PersonalDeviceUsageState_decode_uper;
per_type_encoder_f PersonalDeviceUsageState_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _PersonalDeviceUsageState_H_ */
#include <asn_internal.h>
