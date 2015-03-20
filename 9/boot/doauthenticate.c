
		msg = 0;
		if(readn(afd, tbuf, 2*TICKETLEN) < 0) {
			sprint(error, "%s: %r", pbmsg);
			msg = error;
		}
		break;
	case AuthErr:
		if(readn(afd, error, ERRMAX) < 0) {
			sprint(error, "%s: %r", pbmsg);
			msg = error;
		}
		else {
			error[ERRMAX-1] = 0;
			msg = error;
		}
		break;
	default:
		msg = pbmsg;
		break;
	}

	close(afd);
	return msg;
}

void
doauthenticate(int fd, Method *mp)
{
	char *msg;
	char trbuf[TICKREQLEN];
	char tbuf[2*TICKETLEN];

	print("session...");
	if(fsession(fd, trbuf, sizeof trbuf) < 0)
		fatal("session command failed");

	/* no authentication required? */
	memset(tbuf, 0, 2*TICKETLEN);
	if(trbuf[0] == 0)
		return;

	/* try getting to an auth server */
	print("getting ticket...");
	msg = fromauth(mp, trbuf, tbuf);
	print("authenticating...");
	if(msg == 0)
		if(fauth(fd, tbuf) >= 0)
			return;

	/* didn't work, go for the security hole */
	fprint(2, "no authentication server (%s), using your key as server key\n", msg);
}

char*
checkkey(Method *mp, char *name, char *key)
{
	char *msg;
	Ticketreq tr;
	Ticket t;
	char trbuf[TICKREQLEN];
	char tbuf[TICKETLEN];

	memset(&tr, 0, sizeof tr);
	tr.type = AuthTreq;
	strcpy(tr.authid, name);
	strcpy(tr.hostid, name);
	strcpy(tr.uid, name);
	convTR2M(&tr, trbuf);
	msg = fromauth(mp, trbuf, tbuf);
	if(msg == ccmsg){
		fprint(2, "boot: can't contact auth server, passwd unchecked\n");
		return 0;
	}
	if(msg)
		return msg;
	convM2T(tbuf, &t, key);
	if(t.num == AuthTc && strcmp(name, t.cuid)==0)
		return 0;
	return "no match";
}
