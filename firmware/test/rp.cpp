#include <stdio.h>
#include "pd_try.h"
int main(int c,char**v){FILE*f=fopen(v[1],"r");if(!f){printf("missing\n");return 0;}char l[128];PulseDetector pd;double t=0;int n=0,lo=999,hi=0;while(fgets(l,sizeof l,f)){unsigned long ir,red;if(sscanf(l,"R,%lu,%lu",&ir,&red)!=2)continue;t+=1000.0/103.5;if(pd.update(ir,red,(uint32_t)t)&&pd.bpm){n++;if(pd.bpm<lo)lo=pd.bpm;if(pd.bpm>hi)hi=pd.bpm;}}if(n)printf("%2d readings %d-%d bpm\n",n,lo,hi);else printf("no reading\n");}
