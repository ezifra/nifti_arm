/************************************************************************/
/* Header for CPC Library                                               */
/*                                                                      */
/* Copyright 2000,2001,2002 Dr.-Ing. Thomas W?nsche                     */
/*                                                                      */
/* Company:  EMS Dr. Thomas Wuensche                                    */
/*           Sonnenhang 3                                               */
/*           85304 Ilmmuenster                                          */
/*           Phone: +49-8441-490260                                     */
/*           Fax:   +49-8441-81860                                      */
/*           email: support@ems-wuensche.com                            */
/*           WWW:   www.ems-wuensche.com                                */
/*                                                                      */
/* All rights reserved                                                  */
/*                                                                      */
/* This code is provided "as is" without warranty of any kind, either   */
/* expressed or implied, including but not limited to the liability     */
/* concerning the freedom from material defects, the fitness for        */
/* particular purposes or the freedom of proprietary rights of third    */
/* parties.                                                             */
/************************************************************************/

#ifndef INI_PARSER_H
#define INI_PARSER_H

char *ini_get_key_value(const char *section, const char *skey,
			const char *file, char *dst, size_t size);
int ini_get_section_list(const char *file, char **list, int maxsections);

#endif
