
import json
import os
import cv2
from tqdm import tqdm
from .images import decode_image, fourcc
from .nbs import Decoder


def register(command):
    command.help = "Decode an nbs file and extract any compressed jpeg files into jpeg files"

    # Command arguments
    command.add_argument(
        "files", metavar="files", nargs="+", help="The nbs files to extract the compressed images from"
    )
    command.add_argument("--output", "-o", nargs="?", default=os.getcwd(), help="The folder to extract the images into")


def run(files, output, **kwargs):

    os.makedirs(output, exist_ok=True)
    count = 1

    decoder = Decoder(*files)
    with tqdm(total=len(decoder), unit="B", unit_scale=True, dynamic_ncols=True) as progress:
        for packet in decoder:

            # Update the progress bar
            progress.n = decoder.bytes_read()
            progress.update(0)

            if ((packet.type == "message.output.CompressedImage") and (packet.msg.name == 'Right')):

                data = decode_image(packet.msg.data, packet.msg.format)

                if len(data) == 1:
                    img = data[0]["image"].numpy()
                    fmt = data[0]["fourcc"]
                else:
                    img = [d for d in data if d["name"] == "_colour"][0]["image"].numpy()
                    fmt = data[0]["fourcc"]

                # Debayer if we need to
                if fmt == fourcc("BGGR"):
                    img = cv2.cvtColor(img, cv2.COLOR_BayerBG2RGB)
                elif fmt == fourcc("RGGB"):
                    img = cv2.cvtColor(img, cv2.COLOR_BayerRG2RGB)
                elif fmt == fourcc("GRBG"):
                    img = cv2.cvtColor(img, cv2.COLOR_BayerGR2RGB)
                elif fmt == fourcc("GBRG"):
                    img = cv2.cvtColor(img, cv2.COLOR_BayerGB2RGB)


                filename = os.path.join(output, "{:05d}.jpg".format(count))


                time = packet.msg.timestamp.seconds + packet.msg.timestamp.nanos / 1e9
                cv2.imwrite(filename, img)

                filename = os.path.join(output, "{:05d}.json".format(count))
                count += 1
                with open(
                    filename,
                    "w",
                ) as f:
                    json.dump(
                        {
                            "timestamp": time,
                        },
                        f,
                        indent=4,
                        sort_keys=True,
                    )
