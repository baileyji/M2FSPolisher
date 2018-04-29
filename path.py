

IN_PER_COUNT=0.00024157
COUNTS_PER_IN=1.0/IN_PER_COUNT
MM_PER_COUNT=0.00613100
SPEED_TO_MMM=367.86  #convert cts/ms to mm/min
TWO_PI=6.283185307179586476925286766559

LEM_SIZE=0.5/0.00024157
COUNTS_PER_IN=1/0.00024157
COUNTS_PER_MM=1/0.00613100
PUCK_SIZE=2.0/0.00024157


halfamp = .5* COUNTS_PER_IN
xskew = 0.0
maxpos = (51287, 64102)

startpos=[0,0]
finalpos=[0,0]

startpos[0]=maxpos[0]/20.0
startpos[1]=0.6666667*maxpos[1] + 3.0*COUNTS_PER_MM
finalpos[0]=maxpos[0]*0.6091
dir=1

if startpos[1]>maxpos[1]:
    startpos[1]=maxpos[1]/2
if startpos[1]<0:
    startpos[1]=maxpos[1]/2
finalpos[1]=0

lemsize=halfamp #LEM_SIZE  #in counts
xstretch=xskew #n counts

period=round(13.59867*(lemsize-2069.79))+68
if (period % 2):
    period-=1
period = max(period, 44)
awavenum = TWO_PI/period

#each loop is about loopwid wide the path is finalpos[0]-startpos[0] long
#there are period steps in each so final i is about period*(finalpos[0]-startpos[0])/loopwid
loopwid = (lemsize + xstretch)*0.439696
nlem = abs(finalpos[0]-startpos[0])/loopwid

#From Mathmatica, this is about correct for parameter combinations that make sense
#tight 1.69249 stretched 1.69093 * PI
pathLength=5.3146637*lemsize*nlem
finali=round(period*nlem)


current=startpos
xstretch = (xstretch+lemsize)*dir/14.2899 #go ahead and precompute


def next(i):
    next=copy(startpos)
    t=i * awavenum
    st=sin(t)
    ct=lemsize*cos(t)/(st*st+1.0)
    next[0]+=st*ct + t*xstretch/5
    next[1]+=ct
    return next

points = np.array([next(i) for i in range(202)]).T
plt.plot(points[0]*MM_PER_COUNT,points[1]*MM_PER_COUNT,'.')
