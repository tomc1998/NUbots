
import json
import os

from tqdm import tqdm

from .nbs import Decoder


def register(command):
    command.help = "Decode an nbs file and extract any Motion Capture data into json files"

    # Command arguments
    command.add_argument(
        "files", metavar="files", nargs="+", help="The nbs files to extract the Motion Capture data from"
    )
    command.add_argument("--output", "-o", nargs="?", default=os.getcwd(), help="The folder to extract the Motion Capture data into")


def run(files, output, **kwargs):

    os.makedirs(output, exist_ok=True)
    count = 0
    decoder = Decoder(*files)
    with tqdm(total=len(decoder), unit="B", unit_scale=True, dynamic_ncols=True) as progress:
        for packet in decoder:

            # Update the progress bar
            progress.n = decoder.bytes_read()
            progress.update(0)

            if packet.type == "message.input.MotionCapture":
                count += 1

                RigBod = packet.msg.rigidBodies[0]
                #stuff = packet.msg
                #print("*******************************************")
                #print(count)
                #print("*******************************************")
                #print(stuff)
                with open(
                    os.path.join(
                        output,
                        "{:04d}.json".format(
                            count
                        ),
                    ),
                    "w",
                ) as f:
                    json.dump(
                        {
                            "time_stamp": packet.timestamp,
                            "position": [RigBod.position.x, RigBod.position.y, RigBod.position.z],
                            "rotation": [RigBod.rotation.x, RigBod.rotation.y, RigBod.rotation.z, RigBod.rotation.t],
                            "tracking_valid": RigBod.trackingValid,
                        },
                        f,
                        indent=4,
                        sort_keys=True,
                    )


