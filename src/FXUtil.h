//******************************************************************************
// FXUTIL.H -- FlasherX utility functions
//******************************************************************************
#ifndef FXUTIL_H_
#define FXUTIL_H_


void read_ascii_line( Stream *serial, char *line, int maxbytes );
bool update_firmware( Stream *in, Stream *out,
			uint32_t buffer_addr, uint32_t buffer_size );
void commit_update(Stream *in, Stream *out,uint32_t buffer_addr);

#endif
