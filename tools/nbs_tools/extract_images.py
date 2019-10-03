
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
    Rigid_count = 0
    for packet in decoder.decode(in_file):
        if packet.type == "message.input.MotionCapture":
            MoCap_count += 1
            print("message.input.MotionCapture")
        if packet.type == "message.input.MotionCapture.RigidBody":
            Rigid_count += 1
            print("Message.input.MotionCapture.RigidBody")
        if packet.type == "message.output.CompressedImage":
            image_count += 1
            with open(os.path.join(output_path, "{}_{:012d}.jpg".format(packet.msg.name, packet.timestamp)), "wb") as f:
                f.write(packet.msg.data)
            with open(os.path.join(output_path, "{}_{:012d}.json".format(packet.msg.name, packet.timestamp)), "w") as f:
                Hcw = packet.msg.Hcw
                json.dump(
                    {
                        "Hcw": [
                            [Hcw.x.x, Hcw.x.y, Hcw.x.z, Hcw.x.t],
                            [Hcw.y.x, Hcw.y.y, Hcw.y.z, Hcw.y.t],
                            [Hcw.z.x, Hcw.z.y, Hcw.z.z, Hcw.z.t],
                            [Hcw.t.x, Hcw.t.y, Hcw.t.z, Hcw.t.t],
                        ],
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
        print("Rigid_count:",Rigid_count)
