#!usr/bin/env python

import requests

class MirRequestor:
	def __init__(self,mir_ip="http://192.168.12.20",api_version="v2.0.0"):
		self.header = {"Authorization":"Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="}
		self.url = mir_ip + "/api/" + api_version + "/"

	def abort_all_missions(self):
		end_point = "mission_queue"
		response = requests.delete(self.url+end_point, headers=self.header)
		return response

	def is_mission_done(self, id):
		end_point = "mission_queue/" + str(id)
		response = requests.get(self.url+end_point, headers=self.header).json()
		if (response["state"] == "Done"):
			return True
		else:
			return False


	def dock_pick_point(self):
		end_point = "mission_queue"
		data = {
					"mission_id": "mirconst-guid-0000-0005-actionlist00",  # "Dock"
					"parameters": [
									{
										"id": "Marker",
										"value": "34915c39-1700-11eb-84ba-0001299df266"
									}
								  ]
			   }
		response = requests.post(self.url+end_point, headers=self.header, json=data)
		return response.json()

	def dock_place_point(self):
		end_point = "mission_queue"
		data = {
    					"mission_id": "mirconst-guid-0000-0005-actionlist00",
    					"parameters": [
        							{
            								"id": "Marker",
            								"value": "3041b829-1942-11eb-827e-0001299df266"
       								 }
    							]
			}

		response = requests.post(self.url+end_point, headers=self.header, json=data)
		return response.json()

	def go_standby(self):
		end_point = "mission_queue"
		data = {
					"mission_id": "mirconst-guid-0000-0001-actionlist00",  # "Move"
					"parameters": [
									{
										"id": "Position",
										"value": "3586a1d7-e5a6-11ea-ac58-0001299df266"
									}
								  ]
			   }
		response = requests.post(self.url+end_point, headers=self.header, json=data)
		return response.json()

	def relative_move(self, x=0, y=0, orientation=0):
		end_point = "mission_queue"
		data = {
					"mission_id": "fcf41a69-0e2a-11eb-a64b-0001299df266",  # "RelativeMove"
					"parameters": [
									{
										"id": "x",
										"value": x
									},
									{
										"id": "y",
										"value": y
									},
									{
										"id": "orientation",
										"value": orientation
									}
								  ]
			   }
		response = requests.post(self.url+end_point, headers=self.header, json=data)
		return response.json()

	def robot_pause(self):
		pass

	def robot_play(self):
		pass

	def get_status(self):
		end_point = "status"
		response = requests.get(self.url+end_point,headers=self.header)
		return response.json()



ip = "http://192.168.12.20"
mir = MirRequestor(ip)




