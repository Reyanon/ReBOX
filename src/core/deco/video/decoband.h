/*  The DECO boards' frame drawing in pieces shared with the threads that help the render thread (the driver's
    render_help, as Taito Z's tz_help): the render thread starts the frame's jobs and takes them until none is left,
    a helper takes one at a time; the jobs (bands of lines, or one screen each) draw apart, so the result is the same.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DECOBAND_H
#define RBX_DECOBAND_H

typedef struct {
	int next, left, active, helpers;
	int jobs;
	void (*draw)(int job);
} decoband_t;

// jobs until none is left (one: a single job); the number drawn
static inline int decoband_take(decoband_t *b, int one)
{
	int n = 0;
	for (;;) {
		const int j = __atomic_fetch_add(&b->next, 1, __ATOMIC_ACQ_REL);
		if (j >= b->jobs)
			return n;
		b->draw(j);
		__atomic_sub_fetch(&b->left, 1, __ATOMIC_ACQ_REL);
		n++;
		if (one)
			return n;
	}
}

// the driver's render_help
static inline int decoband_help(decoband_t *b, int helper)
{
	if (!__atomic_load_n(&b->active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&b->helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&b->active, __ATOMIC_SEQ_CST))
		did = decoband_take(b, helper & 0x100) > 0;
	__atomic_sub_fetch(&b->helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

// the driver's render: every job drawn when it returns
static inline void decoband_run(decoband_t *b)
{
	b->next = 0;
	b->left = b->jobs;
	__atomic_store_n(&b->active, 1, __ATOMIC_SEQ_CST);
	decoband_take(b, 0);
	while (__atomic_load_n(&b->left, __ATOMIC_ACQUIRE)) {
	}
	__atomic_store_n(&b->active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&b->helpers, __ATOMIC_SEQ_CST)) {   // (none may still take a job of the next frame)
	}
}
#endif
