#include <u.h>
#include <libc.h>
#include <../boot/boot.h>

/* ID42: 'unknown comment in orig. file, closed here by me...,'
*/
int
plumb(char *dir, char *dest, int *efd, char *here)
{
	char buf[128];
	char name[128];
	int n;

	sprint(name, "%s/clone", dir);
	efd[0] = open(name, ORDWR);
	if(efd[0] < 0)
		return -1;
	n = read(efd[0], buf, sizeof(buf)-1);
	if(n < 0){
		close(efd[0]);
		return -1;
	}
	buf[n] = 0;
	sprint(name, "%s/%s/data", dir, buf);
	if(here){
		sprint(buf, "announce %s", here);
		if(sendmsg(efd[0], buf) < 0){
			close(efd[0]);
			return -1;
		}
	}
	sprint(buf, "connect %s", dest);
	if(sendmsg(efd[0], buf) < 0){
		close(efd[0]);
		return -1;
	}
	efd[1] = open(name, ORDWR);
	if(efd[1] < 0){
		close(efd[0]);
		return -1;
	}
	return efd[1];
}
