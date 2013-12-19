#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../include/cpc_common.h"

enum {
	INI_JUMP_LINE,
	INI_FIND_SEC,
	INI_FOUND_SEC,
	INI_FIND_KEY,
	INI_FOUND_KEY,
	INI_FIND_VALUE,
	INI_FOUND_VALUE
};

#define _ini_jump_line(x) do{ d = fgetc(x); if(d == EOF) { fclose(stream); return NULL;} }while(d !='\n');

HIDE_SYMBOL char *ini_get_key_value(const char *section, const char *skey,
				    const char *file, char *dst,
				    size_t size)
{
	int enState = INI_FIND_SEC, enNextState = INI_FIND_SEC;
	FILE *stream;
	int elemn = 0, v, s, k;
	char c, d, value[32], key[32], sec[32];

	memset(value, 0, sizeof(value));

	stream = fopen(file, "r");
	if (!stream)
		return 0;

	enState = INI_FIND_SEC;

	while ((c = fgetc(stream)) != EOF) {
		switch (c) {
		case ';':
			_ini_jump_line(stream);
			break;
		case ' ':
			continue;
		default:
			switch (enState) {
			case INI_FIND_SEC:
				if (c == '[') {
					enState = INI_FOUND_SEC;
					s = 0;
				}
				break;
			case INI_FOUND_SEC:
				if (c == ']') {
					sec[s] = '\0';
					if (strcmp(sec, section) == 0) {
						//printf("## section: searched %s found %s\n", section, sec);
						_ini_jump_line(stream);
						enState = INI_FIND_KEY;
						k = 0;
					} else {
						enState = INI_FIND_SEC;
					}
					break;
				} else {
					sec[s++] = c;
				}
				break;
			case INI_FIND_KEY:
				if (c == '[' || c == ']') {
					fclose(stream);
					return NULL;
				}
				if (c == '=') {
					key[k] = '\0';
					//printf("## key: searched %s found %s\n", skey, key);
					if (strcmp(key, skey) == 0) {
						enState = INI_FIND_VALUE;
						v = 0;
					} else {
						_ini_jump_line(stream);
						k = 0;
					}
				} else {
					key[k++] = c;
				}
				break;
			case INI_FIND_VALUE:
				if (c == '\n' || c == '\r') {
					value[v] = '\0';
					snprintf(dst, size, "%s", value);
					fclose(stream);
					return dst;
				}
				value[v++] = c;
				break;
			}
			break;
		}
	}
	fclose(stream);

	return NULL;
}

int HIDE_SYMBOL ini_get_section_list(const char *file, char **list,
				     int maxsections)
{
	int enState = INI_FIND_SEC;
	FILE *stream;
	int elemn = 0, s;
	int c;
	char sec[32];

	stream = fopen(file, "r");
	if (!stream)
		return -1;

	enState = INI_FIND_SEC;

	while ((c = fgetc(stream)) != EOF) {
		switch (c) {
		case ';':
			enState = INI_JUMP_LINE;
			break;
		case ' ':
			continue;
		default:
			switch (enState) {
			case INI_JUMP_LINE:
				if (c == '\n' || c == '\r')
					enState = INI_FIND_SEC;
				break;
			case INI_FIND_SEC:
				if (c == '[') {
					enState = INI_FOUND_SEC;
					s = 0;
				}
				break;
			case INI_FOUND_SEC:
				if (c == ']') {
					enState = INI_FIND_SEC;
					sec[s] = '\0';
					list[elemn++] = strdup(sec);
					if (maxsections - elemn <= 0) {
						fclose(stream);
					}
					break;
				} else {
					sec[s++] = c;
				}
				break;
			}
			break;
		}
	}
	fclose(stream);
	return elemn;
}

#if 0
int main(int argc, char **argv)
{
	int i;
	char *list[32];
	char buf[50];

	i = ini_get_section_list(argv[1], &list[0], 32);

	for (i = i - 1; i >= 0; i--) {
		printf("%s\n", list[i]);
		printf("\tType = %s\n",
		       ini_get_key_value(list[i], "InterfaceType", argv[1],
					 buf, sizeof(buf)));
		printf("\tIP = %s\n",
		       ini_get_key_value(list[i], "SlotNr", argv[1], buf,
					 sizeof(buf)));
		free(list[i]);
	}

	return 0;
}
#endif
