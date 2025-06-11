#!usr/bin/env python

from api import *

mir_req = MirRequestor()

mir_req.relative_move(x=0.5, y=0.0, orientation= 0.0)
# mir_req.relative_move(x=1.9601, y=0.3438, orientation= 0.0)
while(1):
    status = mir_req.get_status()
    print(status['state_text'])