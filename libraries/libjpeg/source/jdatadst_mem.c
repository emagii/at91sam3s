/*
 * jdatadst_mem.c
 *
 * Copyright (C) 1994-1996, Thomas G. Lane.
 * Modified 2009 by Guido Vollbeding.
 * This file is part of the Independent JPEG Group's software.
 * For conditions of distribution and use, see the accompanying README file.
 *
 * This file contains compression data destination routines for the case of
 * emitting JPEG data to memory or to a file (or any stdio stream).
 * While these routines are sufficient for most applications,
 * some will want to use a different destination manager.
 * IMPORTANT: we assume that fwrite() will correctly transcribe an array of
 * JOCTETs into 8-bit-wide elements on external storage.  If char is wider
 * than 8 bits on your machine, you may need to do some tweaking.
 */

/* this is not a core library module, so it doesn't define JPEG_INTERNALS */
#include "jinclude.h"
#include "jpeglib.h"
#include "jerror.h"

#ifndef HAVE_STDLIB_H		/* <stdlib.h> should declare malloc(),free() */
extern void * malloc JPP((size_t size));
extern void free JPP((void *ptr));
#endif

#define OUTPUT_BUF_SIZE  4096	/* choose an efficiently fwrite'able size */

/* Expanded data destination object for memory output */

typedef struct {
  struct jpeg_destination_mgr pub; /* public fields */

  unsigned char ** outbuffer;	/* target buffer */
  unsigned long * outsize;
  unsigned char * newbuffer;	/* newly allocated buffer */
  JOCTET * buffer;		/* start of buffer */
  size_t bufsize;
} my_mem_destination_mgr;

typedef my_mem_destination_mgr * my_mem_dest_ptr;

METHODDEF(void)
init_mem_destination (j_compress_ptr cinfo)
{
  /* no work necessary here */
}

METHODDEF(boolean)
empty_mem_output_buffer (j_compress_ptr cinfo)
{
  size_t nextsize;
  JOCTET * nextbuffer;
  my_mem_dest_ptr dest = (my_mem_dest_ptr) cinfo->dest;

  /* Try to allocate new buffer with double size */
  nextsize = dest->bufsize * 2;
  nextbuffer = malloc(nextsize);

  if (nextbuffer == NULL)
    ERREXIT1(cinfo, JERR_OUT_OF_MEMORY, 10);

  MEMCOPY(nextbuffer, dest->buffer, dest->bufsize);

  if (dest->newbuffer != NULL)
    free(dest->newbuffer);

  dest->newbuffer = nextbuffer;

  dest->pub.next_output_byte = nextbuffer + dest->bufsize;
  dest->pub.free_in_buffer = dest->bufsize;

  dest->buffer = nextbuffer;
  dest->bufsize = nextsize;

  return TRUE;
}

METHODDEF(void)
term_mem_destination (j_compress_ptr cinfo)
{
  my_mem_dest_ptr dest = (my_mem_dest_ptr) cinfo->dest;

  *dest->outbuffer = dest->buffer;
  *dest->outsize = dest->bufsize - dest->pub.free_in_buffer;
}

/*
 * Prepare for output to a memory buffer.
 * The caller may supply an own initial buffer with appropriate size.
 * Otherwise, or when the actual data output exceeds the given size,
 * the library adapts the buffer size as necessary.
 * The standard library functions malloc/free are used for allocating
 * larger memory, so the buffer is available to the application after
 * finishing compression, and then the application is responsible for
 * freeing the requested memory.
 */

GLOBAL(void)
jpeg_mem_dest (j_compress_ptr cinfo,
	       unsigned char ** outbuffer, unsigned long * outsize)
{
  my_mem_dest_ptr dest;

  if (outbuffer == NULL || outsize == NULL)	/* sanity check */
    ERREXIT(cinfo, JERR_BUFFER_SIZE);

  /* The destination object is made permanent so that multiple JPEG images
   * can be written to the same buffer without re-executing jpeg_mem_dest.
   */
  if (cinfo->dest == NULL) {	/* first time for this JPEG object? */
    cinfo->dest = (struct jpeg_destination_mgr *)
      (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
				  SIZEOF(my_mem_destination_mgr));
  }

  dest = (my_mem_dest_ptr) cinfo->dest;
  dest->pub.init_destination = init_mem_destination;
  dest->pub.empty_output_buffer = empty_mem_output_buffer;
  dest->pub.term_destination = term_mem_destination;
  dest->outbuffer = outbuffer;
  dest->outsize = outsize;
  dest->newbuffer = NULL;

  if (*outbuffer == NULL || *outsize == 0) {
    /* Allocate initial buffer */
    dest->newbuffer = *outbuffer = malloc(OUTPUT_BUF_SIZE);
    if (dest->newbuffer == NULL)
      ERREXIT1(cinfo, JERR_OUT_OF_MEMORY, 10);
    *outsize = OUTPUT_BUF_SIZE;
  }

  dest->pub.next_output_byte = dest->buffer = *outbuffer;
  dest->pub.free_in_buffer = dest->bufsize = *outsize;
}
