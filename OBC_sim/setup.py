#!/usr/bin/python3

import argparse
import os
import shutil

from msp.conf import main as mspconf
from msp.conf import copy_from

def is_syscommand(opcode):
	return (opcode in range(0x10, 0x20) or opcode in range(0x50, 0x60))
def is_request(opcode):
	return (opcode in range(0x20, 0x30) or opcode in range(0x60, 0x70))
def is_send(opcode):
	return (opcode in range(0x30, 0x40) or opcode in range(0x70, 0x80))

class Command:
	def __init__(self, name, opcode, data=None, printstyle=None):
		self.name = name
		self.opcode = opcode
		self.data = data
		# Set default printstyle
		if printstyle is None and is_request(opcode):
			self.printstyle = "bytes"
		else:
			self.printstyle = "none"
	def __str__(self):
		return str((self.name, self.opcode, self.data, self.printstyle))

class Invoke:
	def __init__(self, command, data=None):
		self.command = command
		self.data = data
class Wait:
	def __init__(self, millis):
		self.millis = millis

class Repeat:
	def __init__(self, value, times):
		self.value = value
		self.times = times

def main():
	parser = argparse.ArgumentParser(description='Setup script for the OBC simulator.')
	parser.add_argument('--config', dest='config_file', type=str, default='config.txt',
	                    help='Name of the config file to be used. Must be in the same'
	                         ' directory as the python script.')
	args = parser.parse_args()

	setup_root = os.path.dirname(os.path.realpath(__file__))
	config_path = os.path.join(setup_root, args.config_file)
	target_path = os.path.join(setup_root, "target")
	obcsim_path = os.path.join(target_path, "obcsim")
	src_path = os.path.join(setup_root, "src")

	lines = scan_file(config_path)
	config = parse(lines)
	hfile = generate_header(config)
	cppfile = generate_sequence(config)

	# Remove existing targets if present
	if 'target' in os.listdir(setup_root):
		if 'obcsim' in os.listdir(target_path):
			shutil.rmtree(obcsim_path)
	os.makedirs(obcsim_path)

	# Create and generate the simulator specific files
	hpath = os.path.join(obcsim_path, "obcsim_configuration.hpp")
	cpppath = os.path.join(obcsim_path, "obcsim_configuration.cpp")
	with open(hpath, "w+") as f:
		f.write(hfile)
	with open(cpppath, "w+") as f:
		f.write(cppfile)
	copy_from(src_path, obcsim_path)

	# Generate the MSP files
	mspargs = argparse.Namespace()
	mspargs.mode = "obc"
	mspargs.lowmem = False
	mspargs.driver = "due"
	mspargs.mtu = None
	mspargs.addr = None
	mspconf(mspargs)

	msp_root = os.path.join(setup_root, "msp")
	msp_target = os.path.join(msp_root, "target")
	copy_from(msp_target, obcsim_path)

	print("MSP Simulator setup completed.")


# Reads the lines from the specified file, removes the comments, and
# concatenates lines with the multiline symbol.
def scan_file(filepath):
	def scan_error(line, msg):
		print('Scan error on line %d: %s' % (line, msg))
		exit(1)

	try:
		with open(filepath) as f:
			lines = []
			lineno = 0
			append_to_last = False
			for line in f:
				lineno += 1

				# Remove comment and check that strings are well formatted.
				in_string = False
				break_point = len(line)
				for i in range(len(line)):
					if in_string:
						if line[i] == "\"":
							in_string = False
					else:
						if line[i] == "\"":
							in_string = True
						elif line[i] == "#":
							# Found a comment symbol
							break_point = i
							break
				if in_string:
					scan_error(lineno, "String missing a closing quotation")
				line = line[:break_point]

				line = line.strip()
				if append_to_last:
					line = lines[-1][1] + " " + line
					del lines[-1]
					append_to_last = False
				
				if len(line) > 0:
					if line[-1] == "\\":
						line = line[:-1].strip()
						append_to_last = True
					lines.append((lineno, line))
			return lines
	except FileNotFoundError:
		print('Configuration file "%s" could not be found.' % filename)
		exit(1)

# Parses the scanned lines and outputs a configuration dictionary.
def parse(lines):
	index = 0
	def parse_error(msg):
		print("Error on line %d in configuration file: %s" % (lines[index][0], msg))
		exit(1)
	def check_error(msg):
		print("Error in configuration file: %s" % msg)
		exit(1)
	def scan_data(data_string):
		if data_string[0:6] == "repeat":
			contents = data_string[6:]
			if contents[0] != "(" and contents[-1] != ")":
				parse_error("Invalid format of repeat")
			params = contents[1:-1].split(",")
			if len(params) != 2:
				parse_error("Invalid number of params to repeat")
			try:
				value = int(params[0], 0)
				times = int(params[1], 0)
			except Exception:
				parse_error("Parameters to repeat must be integers")
			if value not in range(0, 256):
				parse_error("Value to be repeated must be between 0x00 and 0xFF")
			if times < 1:
				parse_error("Value must be repeated a positive number of times")
			values = Repeat(value, times)
		elif data_string[0] == "{" and data_string[-1] == "}":
			values = data_string[1:-1].split(",")
			if len(values) < 1 or values[0] == "":
				parse_error("setdata cannot have an empty array")
			try:
				values = [int(value, 0) for value in values]
				for value in values:
					assert value in range(0, 256)
			except Exception:
				parse_error("All values in array must be integers between 0x00 and 0xFF")
			values = values
		elif data_string[0] == "\"":
			string_data = ""
			in_string = False
			for i in range(len(data_string)):
				if in_string:
					if data_string[i] == "\"":
						in_string = False
					else:
						string_data += data_string[i]
				else:
					if data_string[i] == "\"":
						in_string = True
					elif not data_string[i].isspace():
						parse_error("characters between strings must be spaces")
			if in_string:
				parse_error("string must be closed by a double quotation mark")
			values = string_data
		else:
			parse_error("Invalid data format: %s" % data_string)
		return values

	# Setup configuration dictionary and define default commands
	config = dict()
	config["address"] = None
	config["mtu"] = None
	config["buffersize"] = 4096
	config["errorthreshold"] = 5
	config["commands"] = dict()
	config["commands"]["ACTIVE"] = Command("ACTIVE", 0x10)
	config["commands"]["SLEEP"] = Command("SLEEP", 0x11)
	config["commands"]["POWER_OFF"] = Command("POWER_OFF", 0x12)
	config["commands"]["REQ_PAYLOAD"] = Command("REQ_PAYLOAD", 0x20)
	config["commands"]["REQ_HK"] = Command("REQ_HK", 0x21)
	config["commands"]["REQ_PUS"] = Command("REQ_PUS", 0x22)
	config["commands"]["SEND_TIME"] = Command("SEND_TIME", 0x30)
	config["commands"]["SEND_PUS"] = Command("SEND_PUS", 0x31)
	for command in list(config["commands"].keys()):
		config["commands"][config["commands"][command].opcode] = config["commands"][command]
	config["sequence"] = dict()
	config["sequence"]["init"] = []
	config["sequence"]["loop"] = []

	# Scan options
	while index < len(lines):
		elems = lines[index][1].split()
		# If sequence appears, then we are done scanning the options
		if elems[0] == "sequence":
			break

		if elems[0] == "setaddress":
			if len(elems) != 2:
				parse_error("Wrong number of arguments to setaddress")
			try:
				addr = int(elems[1], 0)
				assert addr in range(0x00, 0x80)
			except Exception:
				parse_error("Address must be an integer between 0x00 and 0x7F")
			config["address"] = addr
		elif elems[0] == "setmtu":
			if len(elems) != 2:
				parse_error("Wrong number of arguments to setmtu")
			try:
				mtu = int(elems[1], 0)
				assert mtu > 0
			except Exception:
				parse_error("MTU must be a positive integer")
			config["mtu"] = mtu
		elif elems[0] == "setbuffersize":
			if len(elems) != 2:
				parse_error("Wrong number of arguments to setbuffersize")
			try:
				bufsize = int(elems[1], 0)
				assert bufsize > 0
			except Exception:
				parse_error("Buffer size must be a positive integer")
			config["buffersize"] = bufsize
		elif elems[0] == "seterrorthreshold":
			if len(elems) != 2:
				parse_error("Wrong number of arguments to seterrorthreshold")
			try:
				threshold = int(elems[1], 0)
				assert threshold > 0
			except Exception:
				parse_error("Error threshold must be a positive integer")
			config["errorthreshold"] = threshold
		elif elems[0] == "addcommand":
			if len(elems) != 3:
				parse_error("Wrong number of arguments to addcommand")

			name = elems[1]
			if name in config["commands"]:
				parse_error("Command \"%s\" already exists" % name)

			try:
				opcode = int(elems[2], 0)
				assert opcode in range(0x50, 0x80)
			except Exception:
				parse_error("Opcode must be a number between 0x50 and 0x7F.")
			if opcode in config["commands"]:
				parse_error("Command \"%s\" already has that opcode." % config["commands"][opcode].name)

			config["commands"][name] = Command(name, opcode)
			config["commands"][opcode] = config["commands"][name]
		elif elems[0] == "setdefaultdata":
			if len(elems) < 3:
				parse_error("Too few arguments to setdefaultdata")

			name = elems[1]
			if name not in config["commands"]:
				parse_error("Command \"%s\" not yet defined." % name)
			if not is_send(config["commands"][name].opcode):
				parse_error("Can only use setdefaultdata for send opcodes")
			
			data = lines[index][1]
			data = data[data.find(elems[0]) + len(elems[0]):]
			data = data[data.find(elems[1]) + len(elems[1]):]
			data = data.strip()
			config["commands"][name].data = scan_data(data)
		elif elems[0] == "setprintstyle":
			if len(elems) != 3:
				parse_error("Invalid number of arguments to setprintstyle")

			name = elems[1]
			if name not in config["commands"]:
				parse_error("Command \"%s\" not yet defined." % name)
			style = elems[2]
			if style not in ["none", "bytes", "bits", "string"]:
				parse_error("Invalid printstyle")

			config["commands"][name].printstyle = style
		else:
			parse_error("Invalid option \"%s\"" % elems[0])

		index += 1

	# Scan sequence
	seqtype = None
	while index < len(lines):
		elems = lines[index][1].split()
		if elems[0] == "sequence":
			if len(elems) != 2:
				parse_error("Invalid number of arguments to sequence")
			if elems[1] == seqtype:
				parse_error("sequence type declared twice")
			if elems[1] == "init" and seqtype is not None:
				parse_error("init must come before loop")
			seqtype = elems[1]
		elif elems[0] == "invoke":
			if len(elems) < 2:
				parse_error("Too few arguments to invoke")
			if seqtype is None:
				parse_error("Sequence type not yet defined")
			name = elems[1]
			if name not in config["commands"]:
				parse_error("Command \"%s\" not found" % name)

			if len(elems) > 2:
				data = lines[index][1]
				data = data[data.find(elems[0]) + len(elems[0]):]
				data = data[data.find(elems[1]) + len(elems[1]):]
				data = data.strip()
				values = scan_data(data)
				if not is_send(config["commands"][name].opcode):
					parse_error("Only send opcodes can have data associated with them")
				config["sequence"][seqtype].append(Invoke(config["commands"][name], values))
			else:
				if is_send(config["commands"][name].opcode) and config["commands"][name].data is None:
					parse_error("Send command \"%s\" has no default data and no data specified by the invoke" % name)
				config["sequence"][seqtype].append(Invoke(config["commands"][name]))
		elif elems[0] == "wait":
			if len(elems) != 2:
				parse_error("Invalid number of arguments to wait")
			if seqtype is None:
				parse_error("Sequence type not yet defined")
			try:
				millis = int(elems[1], 0)
				assert millis > 0
			except Exception:
				parse_error("argument to wait must be a non-negative integer")
			config["sequence"][seqtype].append(Wait(millis))
		else:
			parse_error("Invalid sequence action \"%s\"" % elems[0])

		index += 1

	# Sanity checks
	if config["address"] is None:
		check_error("setaddress must be among the options")
	if config["mtu"] is None:
		check_error("setmtu must be among the options")
	if config["buffersize"] < config["mtu"]:
		check_error("Buffer size must be larger than MTU. (buffersize = %d, mtu = %d)" % (config["buffersize"], config["mtu"]))
	if len(config["sequence"]["init"]) == 0 and len(config["sequence"]["loop"]) == 0:
		check_error("Both init and loop cannot be empty. At least one action must be present")

	# end of parse()
	return config


# Generates the header file with necessary constants
def generate_header(config):
	contents = "/* A generated header file from setup.py */\n\n"
	contents += "#ifndef OBCSIM_CONFIGURATION_H\n"
	contents += "#define OBCSIM_CONFIGURATION_H\n\n"
	contents += "#include \"msp_obc.h\"\n\n"
	contents += "#define EXP_ADDR 0x%02X\n" % config["address"]
	contents += "#define EXP_MTU %d\n" % config["mtu"]
	contents += "#define REQUEST_BUFFER_SIZE %d\n" % config["buffersize"]
	contents += "#define MSP_ERROR_THRESHOLD %d\n\n" % config["errorthreshold"]


	# Add custom opcodes
	for command_name in config["commands"]:
		if type(command_name) is not str:
			continue
		command = config["commands"][command_name]
		contents += "#ifndef MSP_OP_%s\n" % command.name
		contents += "#define MSP_OP_%s 0x%02X\n" % (command.name, command.opcode)
		contents += "#endif\n"
	contents += "\n"

	contents += "void sequence_init(msp_link_t *lnk);\n"
	contents += "void sequence_loop(msp_link_t *lnk);\n\n"

	contents += "#endif /* OBCSIM_CONFIGURATION_H */\n"
	return contents

# Generates a c++ file with all the functions specifying the sequences.
def generate_sequence(config):
	contents = "/* A generated file from setup.py */\n\n"

	contents += "#include \"Arduino.h\"\n"
	contents += "#include \"obcsim_configuration.hpp\"\n"
	contents += "#include \"obcsim_transactions.hpp\"\n\n"

	def seq_data_name(seqtype, index_in_sequence):
		return "seq%s%d" % (seqtype, index_in_sequence)
	def generate_data_array(name, data):
		declaration = "static unsigned char data_%s[] = " % name
		if type(data) is str:
			declaration += "\"%s\";" % data
		elif type(data) is list:
			declaration += "{"
			declaration += ", ".join(["0x%02X" % value for value in data])
			declaration += "};"
		return declaration

	# Generate default data array declarations for send opcodes
	for command_name in config["commands"]:
		if type(command_name) is not str:
			continue
		command = config["commands"][command_name]
		if type(command.data) is list or type(command.data) is str:
			contents += generate_data_array(command.name, command.data) + "\n"

	# Generate data array declarations for invocation specific data
	for seqtype in ["init", "loop"]:
		for i in range(len(config["sequence"][seqtype])):
			action = config["sequence"][seqtype][i]
			if type(action) is not Invoke:
				continue
			if type(action.data) is list or type(action.data) is str:
				contents += generate_data_array(seq_data_name(seqtype, i), action.data) + "\n"
	contents += "\n"

	# Generate the functions for both sequence types
	for seqtype in ["init", "loop"]:
		contents += "void sequence_%s(msp_link_t *lnk)\n" % seqtype
		contents += "{\n"
		for i in range(len(config["sequence"][seqtype])):
			action = config["sequence"][seqtype][i]
			if type(action) is Wait:
				contents += "\tdelay(%d);\n" % action.millis
			elif type(action) is Invoke:
				command = action.command
				opcode_name = "MSP_OP_%s" % command.name
				head = "-------- Invoking %s --------" % command.name
				tail = "-"*len(head)
				contents += "\tSerial.println(\"%s\");\n" % head
				if is_syscommand(command.opcode):
					contents += "\tinvoke_syscommand(lnk, %s);\n" % opcode_name
				elif is_request(command.opcode):
					contents += "\tinvoke_request(lnk, %s, %s);\n" % (opcode_name, str.upper(command.printstyle))
				elif is_send(command.opcode):
					# Check if we are sending an array or just repeating a value
					is_repeat = False
					if action.data is not None:
						if type(action.data) is Repeat:
							is_repeat = True
							repeat_data = action.data
						else:
							data = "data_%s" % seq_data_name(seqtype, i)
					else:
						if type(command.data) is Repeat:
							is_repeat = True
							repeat_data = command.data
						else:
							data = "data_%s" % command.name
					if is_repeat:
						contents += "\tinvoke_send_repeat(lnk, %s, 0x%2X, %d, %s);\n" % (opcode_name, repeat_data.value, repeat_data.times, str.upper(command.printstyle))
					else:
						contents += "\tinvoke_send(lnk, %s, %s, sizeof(%s), %s);\n" % (opcode_name, data, data, str.upper(command.printstyle))
				contents += "\tSerial.println(\"%s\\n\");\n" % tail
		contents += "}\n\n"

	return contents

if __name__ == '__main__':
	main()
