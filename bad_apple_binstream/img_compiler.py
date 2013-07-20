# Dependencies
from PIL import Image
import argparse, os

# Compile an image into a bicolor pixels array
def compileImage(input_filename, fo, threshold, bwMode):

    print 'Processing', input_filename

    # Open the input image file
    img = Image.open(input_filename)
    if not img:
        print 'ERROR: Cannot open the input image file'
        return 1

    # Check image dimensions
    if img.size[0] < 96 or img.size[1] < 64:
        print 'ERROR: Input image is too small'
        return 1

    # Turn the input image into RGB image
    rgb_img = img.convert('RGB')

    # Load pixels of image into memory
    px = rgb_img.load()

    # Prepare binary buffer
    rbuffer = []
    gbuffer = []

    # Process all lines
    for y in range(0, 64):

        # Process all pixels
        for x in range(0, 96, 8):
            
            # Init binary bytes
            rdata = 0
            gdata = 0

            # Turn 8 RGB pixels into 2 binary bytes
            for i in range(0, 8):
                
                # Get RGB values
                r, g, b = px[x + i, y]
                #print x + i, y, '=', r, g, b

                # Compute pixel binary
                if b > threshold and not bwMode:
                    rdata = rdata | 128
                    gdata = gdata | 128
                if r > threshold:
                    rdata = rdata | 128
                if g > threshold and not bwMode:
                    gdata = gdata | 128

                # Shift pixel binary
                if i != 7:
                    rdata = rdata >> 1
                    gdata = gdata >> 1

            # Store binary
            #print x, y, '=', rdata, gdata
            rbuffer.append(chr(rdata ^ 255))
            gbuffer.append(chr(gdata ^ 255))

    # Save buffer to file
    fo.write(''.join(rbuffer))
    fo.write(''.join(gbuffer))

    # No error
    return 0

def processFiles(filenames, threshold, bwMode):
    # Process each images
    for filename in filenames:
        compileImage(filename, fo, threshold, bwMode)

def processDirectories(directories, threshold, bwMode):
    # Process each images
    for d in directories:
        for f in os.listdir(d):
            current_file = os.path.join(d, f)
            compileImage(current_file, fo, threshold, bwMode)

# Create a new arguments parser
parser = argparse.ArgumentParser(description = 'Turn image(s) into binary dump file for the skywodd led matrix project')
parser.add_argument('input_filenames', metavar='INPUT FILE', nargs='+', help='Input image filename(s) to process')
parser.add_argument('output_filename', metavar='OUTPUT FILE', help='Output filename for the binary file')
parser.add_argument('-d', '--directory', dest='process', action='store_const', const=processDirectories, default=processFiles, help='Process a whole directory instead of files.')
parser.add_argument('-t', '--threshold', dest='threshold', action='store', type=int, default=127, help='Threshold level for each color channel.')
parser.add_argument('-m', '--monocolor', dest='bwMode', action='store_const', const=True, default=False, help='Black and white mode')

# Parse CLI arguments
args = parser.parse_args()

# Open the output binary file
fo = open(args.output_filename, 'wb')
if not fo:
    print 'ERROR: Cannot open the output file'
    exit(1)

# Process input
args.process(args.input_filenames, args.threshold, args.bwMode)

# Close the input and output file
fo.close()
