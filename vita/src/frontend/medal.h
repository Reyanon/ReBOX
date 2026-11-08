// Medal cabinet panels beside the game picture (Taito O System medal games), medal.c
#ifndef RBX_FRONTEND_MEDAL_H
#define RBX_FRONTEND_MEDAL_H

int rbx_medal_game(void);   // the running game is a medal game
void rbx_medal_start(void); // after the game started (video set up)
void rbx_medal_stop(void);  // at the game's end
void rbx_medal_tick(void);  // once per frame, after the picture was drawn
void rbx_medal_touch(void); // once per frame after rbx_input_poll: touching the panel presses the cabinet buttons

#endif
