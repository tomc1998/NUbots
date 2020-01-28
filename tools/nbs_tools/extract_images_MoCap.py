
import json
import os

from tqdm import tqdm

from .nbs import Decoder

def register(command):
    command.help = "Decode an nbs file and extract any Motion Capture data into json files"

    # Command arguments
    command.add_argument(
        "files", metavar="files", nargs="+", help="The nbs file to extract the compressed images and MoCap data from"
    )
    command.add_argument("--output", "-o", nargs="?", default=os.getcwd(), help="The folder to extract the images and json files into")


def run(files, output, **kwargs):

    os.makedirs(output, exist_ok=True)
    image_count = 0
    MoCap_count = 0
    decoder = Decoder(*files)
    with tqdm(total=len(decoder), unit="B", unit_scale=True, dynamic_ncols=True) as progress:
        for packet in decoder:

            # Update the progress bar
            progress.n = decoder.bytes_read()
            progress.update(0)

            if packet.type == "message.input.MotionCapture":
                MoCap_count += 1
                RigBod = packet.msg.rigidBodies[0]
                with open(os.path.join(output, "{:012d}.json".format(packet.timestamp)), "w") as f:
                    json.dump(
                        {
                            "packet_time_stamp": packet.timestamp,
                            "MoCap_time_stamp": packet.msg.timestamp,
                            "position": [RigBod.position.x, RigBod.position.y, RigBod.position.z],
                            "rotation": [RigBod.rotation.x, RigBod.rotation.y, RigBod.rotation.z, RigBod.rotation.t],
                            "tracking_valid": RigBod.trackingValid,
                        },
                        f,
                        indent=4,
                        sort_keys=True,
                    )

            # Left eye only
            elif packet.type == "message.output.CompressedImage":
                image_count += 1
                with open(os.path.join(output, "{:012d}.jpg".format(packet.timestamp)), "wb") as f:
                    f.write(packet.msg.data)
                #with open(os.path.join(output, "{:012d}.txt".format(packet.timestamp)), "w") as f:
                #    f.write(str(int(packet.msg.timestamp.seconds * 1e9 + packet.msg.timestamp.nanos)))



    print("image_count:",image_count)
    print("MoCap_count:",MoCap_count)

