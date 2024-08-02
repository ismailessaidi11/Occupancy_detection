/*
 * syscall.h
 *
 *  Created on: Jul 30, 2024
 *      Author: Guest2
 */

#ifndef INC_SYSCALL_H_
#define INC_SYSCALL_H_


#include <errno.h>
#include <stdio.h>
#include <sys/stat.h>

int _getpid(void);
int _kill(int pid, int sig);
void _exit (int status);
__attribute__((weak)) int _read(int file, char *ptr, int len);
__attribute__((weak)) int _write(int file, char *ptr, int len);
int _close(int file);
int _isatty(int file);
int _lseek(int file, int ptr, int dir);
int _open(char *path, int flags, ...);
int _wait(int *status);
int _unlink(char *name);
int _link(char *old, char *new);
int _fork(void);
int _execve(char *name, char **argv, char **env);
int _fstat(int file, struct stat *st);


#endif /* INC_SYSCALL_H_ */
