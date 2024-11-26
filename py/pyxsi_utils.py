# Copyright (C) 2024, Advanced Micro Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of pyxsi nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import errno
import os
import os.path
import pyxsi
import subprocess
import sys
from typing import Optional

axilite_expected_signals = [
    "AWVALID",
    "AWREADY",
    "AWADDR",
    "WVALID",
    "WREADY",
    "WDATA",
    "WSTRB",
    "ARVALID",
    "ARREADY",
    "ARADDR",
    "RVALID",
    "RREADY",
    "RDATA",
    "RRESP",
    "BVALID",
    "BREADY",
    "BRESP",
]

aximm_expected_signals = [
    "AWVALID",
    "AWREADY",
    "AWADDR",
    "AWID",
    "AWLEN",
    "AWSIZE",
    "AWBURST",
    "AWLOCK",
    "AWCACHE",
    "AWPROT",
    "AWQOS",
    "AWREGION",
    "AWUSER",
    "WVALID",
    "WREADY",
    "WDATA",
    "WSTRB",
    "WLAST",
    "WID",
    "WUSER",
    "ARVALID",
    "ARREADY",
    "ARADDR",
    "ARID",
    "ARLEN",
    "ARSIZE",
    "ARBURST",
    "ARLOCK",
    "ARCACHE",
    "ARPROT",
    "ARQOS",
    "ARREGION",
    "ARUSER",
    "RVALID",
    "RREADY",
    "RDATA",
    "RLAST",
    "RID",
    "RUSER",
    "RRESP",
    "BVALID",
    "BREADY",
    "BRESP",
    "BID",
    "BUSER",
]

def launch_process_helper(args, proc_env=None, cwd=None):
    """Helper function to launch a process in a way that facilitates logging
    stdout/stderr with Python loggers.
    Returns (cmd_out, cmd_err)."""
    if proc_env is None:
        proc_env = os.environ.copy()
    with subprocess.Popen(
        args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=proc_env, cwd=cwd
    ) as proc:
        (cmd_out, cmd_err) = proc.communicate()
    if cmd_out is not None:
        cmd_out = cmd_out.decode("utf-8")
        sys.stdout.write(cmd_out)
    if cmd_err is not None:
        cmd_err = cmd_err.decode("utf-8")
        sys.stderr.write(cmd_err)
    return (cmd_out, cmd_err)

def locate_glbl() -> Optional[str]:
    """
    Tries to determine the glbl.v file path from environment variables.
    Returns None if it cannot be found.
    """
    # Get GLBL from the Vitis environment variable
    vivado_path = os.environ.get('XILINX_VIVADO')
    if vivado_path:
        glbl_path = os.path.join(vivado_path, 'data', 'verilog', 'src', 'glbl.v')
        if os.path.isfile(glbl_path):
            return glbl_path
    return None


def compile_sim_obj(top_module_name, source_list, sim_out_dir):
    # create a .prj file with the source files
    with open(sim_out_dir + "/rtlsim.prj", "w") as f:

        glbl = locate_glbl()
        if glbl is not None:
            f.write(f"verilog work {glbl}\n")

        for src_line in source_list:
            if src_line.endswith(".v"):
                f.write(f"verilog work {src_line}\n")
            elif src_line.endswith(".vhd"):
                f.write(f"vhdl2008 work {src_line}\n")
            elif src_line.endswith(".sv"):
                f.write(f"sv work {src_line}\n")
            elif src_line.endswith(".vh"):
                # skip adding Verilog headers
                continue
            else:
                raise Exception(f"Unknown extension for .prj file sources: {src_line}")

    # now call xelab to generate the .so for the design to be simulated
    # TODO make debug controllable to allow faster sim when desired
    # list of libs for xelab retrieved from Vitis HLS cosim cmdline
    # the particular lib version used depends on the Vivado/Vitis version being used
    # but putting in multiple (nonpresent) versions seems to pose no problem as long
    # as the correct one is also in there. at least this is how Vitis HLS cosim is
    # handling it.
    # TODO make this an optional param instead of hardcoding
    xelab_libs = [
        "smartconnect_v1_0", "axi_protocol_checker_v1_1_12", "axi_protocol_checker_v1_1_13", 
        "axis_protocol_checker_v1_1_11", "axis_protocol_checker_v1_1_12", "xil_defaultlib", 
        "unisims_ver", "xpm", "floating_point_v7_1_16", "floating_point_v7_0_21", "floating_point_v7_1_18",
        "floating_point_v7_1_15", "floating_point_v7_1_19", 
    ]

    cmd_xelab = [
        "xelab",
        "work." + top_module_name,
        "-relax",
        "-prj",
        "rtlsim.prj",
        "-debug",
        "all",
        "-dll",
        "-s",
        top_module_name,
    ]
    for lib in xelab_libs:
        cmd_xelab.append("-L")
        cmd_xelab.append(lib)

    if locate_glbl() is not None:
        cmd_xelab.insert(1, "work.glbl")

    launch_process_helper(cmd_xelab, cwd=sim_out_dir)
    out_so_relative_path = "xsim.dir/%s/xsimk.so" % top_module_name
    out_so_full_path = sim_out_dir + "/" + out_so_relative_path

    if not os.path.isfile(out_so_full_path):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), out_so_full_path)

    return (sim_out_dir, out_so_relative_path)


def load_sim_obj(sim_out_dir, out_so_relative_path, tracefile=None, is_toplevel_verilog=True):
    vivado_path = os.environ.get('XILINX_VIVADO')
    # xsi kernel lib name depends on Vivado version (renamed in 2024.2)
    try:
        if vivado_path:
            if float(vivado_path.split("/")[-1]) > 2024.1:
                simkernel_so = "libxv_simulator_kernel.so"
            else:
                simkernel_so = "librdi_simulator_kernel.so"
        else:
            simkernel_so = "librdi_simulator_kernel.so"
    except Exception as e:
        # fallback/default
        simkernel_so = "librdi_simulator_kernel.so"
    oldcwd = os.getcwd()
    os.chdir(sim_out_dir)
    sim = pyxsi.XSI(
        out_so_relative_path,
        simkernel_so,
        is_toplevel_verilog=is_toplevel_verilog,
        tracefile=tracefile,
        logfile="rtlsim.log",
    )
    os.chdir(oldcwd)
    return sim


def _find_signal(sim, signal_name):
    signal_list = [sim.get_port_name(i) for i in range(sim.get_port_count())]
    # handle both mixed caps and lowercase signal names
    if signal_name in signal_list:
        return signal_name
    elif signal_name.lower() in signal_list:
        return signal_name.lower()
    else:
        return None


def _read_signal(sim, signal_name):
    signal_name = _find_signal(sim, signal_name)
    port_val = sim.get_port_value(signal_name)
    return int(port_val, 2)


def _write_signal(sim, signal_name, signal_value):
    signal_name = _find_signal(sim, signal_name)
    signal_len = len(sim.get_port_value(signal_name))
    if signal_value < 0:
        raise Exception("TODO: _write_signal needs fix for 2s complement neg values")
    signal_bin_value = f"{signal_value:0{signal_len}b}"[-signal_len:]
    sim.set_port_value(signal_name, signal_bin_value)


def reset_rtlsim(sim, rst_name="ap_rst_n", active_low=True, clk_name="ap_clk", clk2x_name="ap_clk2x"):
    _write_signal(sim, clk_name, 1)
    if not (_find_signal(sim, clk2x_name) is None):
        _write_signal(sim, clk2x_name, 1)
    _write_signal(sim, rst_name, 0 if active_low else 1)
    for _ in range(2):
        toggle_clk(sim, clk_name, clk2x_name)

    _write_signal(sim, rst_name, 1 if active_low else 0)
    toggle_clk(sim, clk_name, clk2x_name)
    toggle_clk(sim, clk_name, clk2x_name)


def toggle_clk(sim, clk_name="ap_clk", clk2x_name="ap_clk2x"):
    toggle_neg_edge(sim, clk_name=clk_name, clk2x_name=clk2x_name)
    toggle_pos_edge(sim, clk_name=clk_name, clk2x_name=clk2x_name)


def toggle_neg_edge(sim, clk_name="ap_clk", clk2x_name="ap_clk2x"):
    if not (_find_signal(sim, clk2x_name) is None):
        _write_signal(sim, clk_name, 0)
        _write_signal(sim, clk2x_name, 1)
        sim.run(5000)
        _write_signal(sim, clk2x_name, 0)
        sim.run(5000)
    else:
        _write_signal(sim, clk_name, 0)
        sim.run(5000)    


def toggle_pos_edge(sim, clk_name="ap_clk", clk2x_name="ap_clk2x", signals_to_write={}):
    if not (_find_signal(sim, clk2x_name) is None):
        _write_signal(sim, clk_name, 1)
        _write_signal(sim, clk2x_name, 1)
        sim.run(5000)
        for k, v in signals_to_write.items():
            _write_signal(sim, k, v)
        _write_signal(sim, clk2x_name, 0)
        sim.run(5000)
    else:
        _write_signal(sim, clk_name, 1)
        sim.run(5000)
        for k, v in signals_to_write.items():
            _write_signal(sim, k, v)

def close_rtlsim(sim):
    sim.close()

def rtlsim_multi_io(
    handle,
    io_dict,
    num_out_values,
    sname="_V_V_",
    liveness_threshold=10000,
    hook_preclk=None,
    hook_postclk=None,
    clk_name="ap_clk",
    clk2x_name="ap_clk2x",
):
    for outp in io_dict["outputs"]:
        _write_signal(handle, outp + sname + "TREADY", 1)

    # observe if output is completely calculated
    # total_cycle_count will contain the number of cycles the calculation ran
    output_done = False
    total_cycle_count = 0
    output_count = 0
    old_output_count = 0

    # avoid infinite looping of simulation by aborting when there is no change in
    # output values after 100 cycles
    no_change_count = 0

    while not (output_done):
        signals_to_write = {}
        if hook_preclk:
            hook_preclk(handle)
        # Toggle falling edge to arrive at a delta cycle before the rising edge
        toggle_neg_edge(handle, clk_name=clk_name, clk2x_name=clk2x_name)

        # examine signals, decide how to act based on that but don't update yet
        # so only read_signal access in this block, no _write_signal
        for inp in io_dict["inputs"]:
            inputs = io_dict["inputs"][inp]
            signal_name = inp + sname
            if (
                _read_signal(handle, signal_name + "TREADY") == 1
                and _read_signal(handle, signal_name + "TVALID") == 1
            ):
                inputs = inputs[1:]
            io_dict["inputs"][inp] = inputs

        for outp in io_dict["outputs"]:
            outputs = io_dict["outputs"][outp]
            signal_name = outp + sname
            if (
                _read_signal(handle, signal_name + "TREADY") == 1
                and _read_signal(handle, signal_name + "TVALID") == 1
            ):
                outputs = outputs + [_read_signal(handle, signal_name + "TDATA")]
                output_count += 1
            io_dict["outputs"][outp] = outputs

        # update signals based on decisions in previous block, but don't examine anything
        # so only write_signal access in this block, no read_signal
        for inp in io_dict["inputs"]:
            inputs = io_dict["inputs"][inp]
            signal_name = inp + sname
            signals_to_write[signal_name + "TVALID"] = 1 if len(inputs) > 0 else 0
            signals_to_write[signal_name + "TDATA"] = inputs[0] if len(inputs) > 0 else 0

        # Toggle rising edge to arrive at a delta cycle before the falling edge
        toggle_pos_edge(handle, clk_name=clk_name, clk2x_name=clk2x_name, signals_to_write=signals_to_write)

        if hook_postclk:
            hook_postclk(handle)

        total_cycle_count = total_cycle_count + 1

        if output_count == old_output_count:
            no_change_count = no_change_count + 1
        else:
            no_change_count = 0
            old_output_count = output_count

        # check if all expected output words received
        if output_count == num_out_values:
            output_done = True

        # end sim on timeout
        if no_change_count == liveness_threshold:
            raise Exception(
                "Error in simulation! Takes too long to produce output. "
                "Consider setting the liveness_threshold parameter to a "
                "larger value."
            )

    return total_cycle_count

def wait_for_handshake(sim, ifname, basename="s_axi_control_", dataname="DATA"):
    """Wait for handshake (READY and VALID high at the same time) on given
    interface on pyxsi sim object.

    Arguments:
    - sim : pyxsi sim object
    - ifname : name for decoupled interface to wait for handshake on
    - basename : prefix for decoupled interface name
    - dataname : interface data sig name, will be return value if it exists

    Returns: value of interface data signal during handshake (if given by dataname),
    None otherwise (e.g. if there is no data signal associated with interface)
    """
    ret = None
    while 1:
        hs = (
            _read_signal(sim, basename + ifname + "READY") == 1
            and _read_signal(sim, basename + ifname + "VALID") == 1
        )
        try:
            ret = _read_signal(sim, basename + ifname + dataname)
        except Exception:
            ret = None
        toggle_clk(sim)
        if hs:
            break
    return ret


def multi_handshake(sim, ifnames, basename="s_axi_control_"):
    """Perform a handshake on list of interfaces given by ifnames. Will assert
    VALID and de-assert after READY observed, in as few cycles as possible."""

    done = []
    for ifname in ifnames:
        _write_signal(sim, basename + ifname + "VALID", 1)
    while len(ifnames) > 0:
        for ifname in ifnames:
            if (
                _read_signal(sim, basename + ifname + "READY") == 1
                and _read_signal(sim, basename + ifname + "VALID") == 1
            ):
                done.append(ifname)
        toggle_clk(sim)
        for ifname in done:
            if ifname in ifnames:
                ifnames.remove(ifname)
            _write_signal(sim, basename + ifname + "VALID", 0)


def axilite_write(sim, addr, val, basename="s_axi_control_", wstrb=0xF, sim_addr_and_data=True):
    """Write val to addr on AXI lite interface given by basename.

    Arguments:
    - sim : PyVerilator sim object
    - addr : address for write
    - val : value to be written at addr
    - basename : prefix for AXI lite interface name
    - wstrb : write strobe value to do partial writes, see AXI protocol reference
    - sim_addr_and_data : handshake AW and W channels simultaneously
    """
    _write_signal(sim, basename + "WSTRB", wstrb)
    _write_signal(sim, basename + "WDATA", val)
    _write_signal(sim, basename + "AWADDR", addr)
    if sim_addr_and_data:
        multi_handshake(sim, ["AW", "W"], basename=basename)
    else:
        _write_signal(sim, basename + "AWVALID", 1)
        wait_for_handshake(sim, "AW", basename=basename)
        # write request done
        _write_signal(sim, basename + "AWVALID", 0)
        # write data
        _write_signal(sim, basename + "WVALID", 1)
        wait_for_handshake(sim, "W", basename=basename)
        # write data OK
        _write_signal(sim, basename + "WVALID", 0)
    # wait for write response
    _write_signal(sim, basename + "BREADY", 1)
    wait_for_handshake(sim, "B", basename=basename)
    # write response OK
    _write_signal(sim, basename + "BREADY", 0)


def axilite_read(sim, addr, basename="s_axi_control_"):
    """Read val from addr on AXI lite interface given by basename.

    Arguments:
    - sim : PyVerilator sim object
    - addr : address for read
    - basename : prefix for AXI lite interface name

    Returns: read value from AXI lite interface at given addr
    """
    _write_signal(sim, basename + "ARADDR", addr)
    _write_signal(sim, basename + "ARVALID", 1)
    wait_for_handshake(sim, "AR", basename=basename)
    # read request OK
    _write_signal(sim, basename + "ARVALID", 0)
    # wait for read response
    _write_signal(sim, basename + "RREADY", 1)
    ret_data = wait_for_handshake(sim, "R", basename=basename)
    _write_signal(sim, basename + "RREADY", 0)
    return ret_data

