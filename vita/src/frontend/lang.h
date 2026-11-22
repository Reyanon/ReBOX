// The menus' language: ux0:data/rebox/lang/<code>.json (a map of the English texts to their translations) and the
// releases' changes v<version>-<code>.json. The text drawing functions (ui.c) look every text up (ui_tr), so a text
// drawn as it is needs no change where it is drawn; a text made with snprintf has its format looked up (T).
// GPL-2.0-or-later.
#ifndef RBX_FRONTEND_LANG_H
#define RBX_FRONTEND_LANG_H

typedef struct {
	char code[8];       // "kor"
	char name[32];      // in the language itself: "한국어"
	char english[32];   // its English name, the order of the list: "Korean"
} rbx_lang;

void rbx_lang_init(void);                // (start) the missing files installed, the languages found, g_settings.language used
void rbx_lang_use(const char *code);     // another language ("eng": none)
int rbx_lang_count(void);
int rbx_lang_english(void);              // no translation in use (English: its plural "s" added by the code)
const rbx_lang *rbx_lang_at(int i);      // by their English names
int rbx_lang_index(const char *code);    // -1: none

const char *ui_tr(const char *s);        // the text in the language in use (itself when it has no translation)
#define T(s) ui_tr(s)

#endif
