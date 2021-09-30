from pymongo import MongoClient
import time
import json


class Logger:

    def __init__(self, *args):
        with open('logging.json', 'r') as json_file:
            data = json.load(json_file)
            json_file.close()
            self.client = MongoClient(data["uri"])

            self.collection_flight = self.client[data["db"]][data["collection_flight"]]
            self.bulk_index = data["bulk_index"]
            self.mongo_docs = []
            self.total_docs = 0
            self.p_id = data["p_id"]
            self.t_id = data["t_id"]

            print("Logging started: PID:", self.p_id, "TID:", self.t_id)

            data["t_id"] = self.t_id + 1
            updated_json = open('logging.json', 'w')
            json.dump(data,updated_json)
            updated_json.close()

    def log_frame(self,
                  target_x_pos, target_y_pos, target_z_pos,
                  target_x_rot, target_y_rot, target_z_rot,
                  drone_name, drone_x_pos, drone_y_pos, drone_z_pos,
                  drone_x_rot, drone_y_rot, drone_z_rot):

        # create timestamp
        millis = int(round(time.time() * 1000))
        # create doc_body
        doc_body = {
            "participant_id": self.p_id,
            "test_id": self.t_id,
            "timestamp": millis,

            "target_x_pos": target_x_pos,
            "target_y_pos": target_y_pos,
            "target_z_pos": target_z_pos,

            "target_x_rot": target_x_rot,
            "target_y_rot": target_y_rot,
            "target_z_rot": target_z_rot,

            "drone_name": drone_name,

            "drone_x_pos": drone_x_pos,
            "drone_y_pos": drone_y_pos,
            "drone_z_pos": drone_z_pos,

            "drone_x_rot": drone_x_rot,
            "drone_y_rot": drone_y_rot,
            "drone_z_rot": drone_z_rot,
        }

        self.mongo_docs.append(doc_body)
        if len(self.mongo_docs) >= self.bulk_index:
            self.collection_flight.insert_many(self.mongo_docs)
            self.mongo_docs = []
