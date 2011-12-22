/*
 *	main.h
 */

#if	!defined(_MAIN_H)
#define	_MAIN_H
void	header(void);
int	init_NAND_flash (void);
int	mount_disk(FATFS fs);
int	erase_disk(void);
int	open_dir(const char *directory);
int	format_disk(void);
int	create_file(FIL *FileObject, const char *filename);
int	open_file(FIL *FileObject, const char *filename);
int	write_firmware(FIL *FileObject);
int	close_file(FIL *FileObject);
int	verify(const char *filename, const char *original, unsigned int size);
#endif

