print 'setPixelAt(FB_GREEN, i, 63, 1);'
for i in range(62, -1, -1):
    if i > 7:
        print "setPixelAt(FB_GREEN, i, %d, s > %d);" % (i, 256 - ((i + 1) * 4))
    if i <= 23:
        print "setPixelAt(FB_RED, i, %d, s > %d);" % (i, 256 - ((i + 1)* 4))
