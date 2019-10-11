
import os
import json
from . import decoder


def register(command):
    command.help = "Decode an nbs file and extract any compressed jpeg files into jpeg files"

    # Command arguments
    command.add_argument("in_file", metavar="in_file", help="The nbs file to extract the compressed images from")
    command.add_argument(
        "out_path", nargs="?", default=os.getcwd(), metavar="out_path", help="The folder to extract the images into"
    )


def run(in_file, out_path, **kwargs):

    output_path = os.path.join(out_path, os.path.basename(os.path.splitext(in_file)[0]))
    os.makedirs(output_path, exist_ok=True)
    image_count = 0
    MoCap_count = 0
    newMoCapData = False
    for packet in decoder.decode(in_file):
        if packet.type == "message.input.MotionCapture":
            MoCap_count += 1
            newMoCapData = True
            RigBod = packet.msg.rigidBodies[0]
            #print(RigBod.position)

        # Left eye only
        elif ((newMoCapData == True) and (packet.type == "message.output.CompressedImage") and (packet.msg.name == '01188260')):
            newMoCapData = False
            image_count += 1
            with open(os.path.join(output_path, "{}_{:012d}.jpg".format(packet.msg.name, packet.timestamp)), "wb") as f:
                f.write(packet.msg.data)
            with open(os.path.join(output_path, "{}_{:012d}.json".format(packet.msg.name, packet.timestamp)), "w") as f:
                # Data goes here
                json.dump(
                    {
                        "time_stamp": packet.timestamp,
                        "position": [RigBod.position.x, RigBod.position.y, RigBod.position.z],
                        "rotation": [RigBod.rotation.x, RigBod.rotation.y, RigBod.rotation.z, RigBod.rotation.t],
                        "tracking_valid": RigBod.trackingValid,
                        "lens": {
                            "projection": packet.msg.lens.projection,
                            "focal_length": packet.msg.lens.focal_length,
                            "centre": [0, 0],
                            "fov": packet.msg.lens.fov.x,
                        },
                    },
                    f,
                    indent=4,
                    sort_keys=True,
                )
    print("image_count:",image_count)
    print("MoCap_count:",MoCap_count)
