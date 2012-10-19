/**
 * Copyright (c) 2012, Regents of the University of California
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *   Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *   Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/

package edu.berkeley.path.beats;

import edu.berkeley.path.beats.simulator.SiriusErrorLog;

/**
 * Implements "Sirius: Concept of Operations"
 */
public class Runner {

	/**
	 * @param args command-line arguments
	 */
	public static void main(String[] args) {
		try {
			if (0 == args.length) throw new InvalidUsageException();
			String cmd = args[0];
			String[] arguments = new String[args.length - 1];
			System.arraycopy(args, 1, arguments, 0, args.length - 1);
			if (cmd.equals("import") || cmd.equals("i")) {
				if (arguments.length != 1) throw new InvalidUsageException("Usage: import|i scenario_file_name");
				edu.berkeley.path.beats.db.importer.ScenarioLoader.load(arguments[0]);
			} else if (cmd.equals("update") || cmd.equals("u")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("export") || cmd.equals("e")) {
				if (0 == arguments.length || 2 < arguments.length)
					throw new InvalidUsageException("Usage: export|e scenario_id [output_file_name]");
				else {
					String filename = 1 == arguments.length ? arguments[0] + ".xml" : arguments[1];
					edu.berkeley.path.beats.db.exporter.ScenarioRestorer.export(Integer.parseInt(arguments[0]), filename);
				}
			} else if (cmd.equals("calibrate") || cmd.equals("c")) {
				edu.berkeley.path.beats.calibrator.FDCalibrator.main(arguments);
			} else if (cmd.equals("simulate") || cmd.equals("s")) {
				edu.berkeley.path.beats.simulator.Runner.run_db(arguments);
			} else if (cmd.equals("simulate_output") || cmd.equals("so")) {
				edu.berkeley.path.beats.simulator.Runner.main(arguments);
			} else if (cmd.equals("debug")) {
				edu.berkeley.path.beats.simulator.Runner.debug(arguments);
			} else if (cmd.equals("simulate_process") || cmd.equals("sp")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("list_scenarios") || cmd.equals("ls")) {
				edu.berkeley.path.beats.db.Lister.listScenarios();
			} else if (cmd.equals("list_runs") || cmd.equals("lr")) {
				if (1 == arguments.length)
					edu.berkeley.path.beats.db.Lister.listRuns(Long.parseLong(arguments[0], 10));
				else
					throw new InvalidUsageException("Usage: list_runs|lr scenario_id");
			} else if (cmd.equals("load") || cmd.equals("l")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("process") || cmd.equals("p")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("output") || cmd.equals("o")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("list_aggregations") || cmd.equals("la")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("link_data") || cmd.equals("ld")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("route_data") || cmd.equals("rd")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("node_data") || cmd.equals("nd")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("signal_data") || cmd.equals("sd")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("detection_data") || cmd.equals("dd")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("probe_data") || cmd.equals("pd")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("controller_data") || cmd.equals("cd")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("report") || cmd.equals("r")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("init")) {
				edu.berkeley.path.beats.db.Admin.init();
			} else if (cmd.equals("clear_data") || cmd.equals("cld")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("clear_processed") || cmd.equals("clp")) {
				if (1 == arguments.length)
					edu.berkeley.path.beats.db.Cleaner.clearProcessed(Long.parseLong(arguments[0], 10));
				else
					throw new InvalidUsageException("Usage: clear_processed|clp scenario_id");
			} else if (cmd.equals("clear_scenario") || cmd.equals("cls")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("clear_all") || cmd.equals("cla")) {
				throw new NotImplementedException(cmd);
			} else if (cmd.equals("version") || cmd.equals("v")) {
				printVersion();
			} else if (cmd.equals("convert_units") || cmd.equals("cu")) {
				if (2 == arguments.length)
					edu.berkeley.path.beats.util.UnitConverter.convertUnits(arguments[0], arguments[1]);
				else
					throw new InvalidUsageException("Usage: convert_units|cu input_file output_file");
			} else throw new InvalidCommandException(cmd);
			if (SiriusErrorLog.haserror()) SiriusErrorLog.print();
		} catch (InvalidUsageException exc) {
			String msg = exc.getMessage();
			if (null == msg) msg = "Usage: command [parameters]";
			System.err.println(msg);
			System.exit(1);
		} catch (NotImplementedException exc) {
			System.err.println(exc.getMessage());
			System.exit(1);
		} catch (InvalidCommandException exc) {
			System.err.println(exc.getMessage());
			System.exit(1);
		} catch (Exception exc) {
			exc.printStackTrace();
			System.exit(2);
		} finally {
			if (edu.berkeley.path.beats.db.Service.isInit())
				edu.berkeley.path.beats.db.Service.shutdown();
		}
	}

	@SuppressWarnings("serial")
	public static class NotImplementedException extends Exception {
		/**
		 * Constructs a <code>NotImplementedException</code> for the specified command
		 * @param cmd name of the command
		 */
		NotImplementedException(String cmd) {
			super("Command '" + cmd + "' is not implemented");
		}
	}

	@SuppressWarnings("serial")
	public static class InvalidCommandException extends Exception {
		/**
		 * Constructs an <code>InvalidCommandException</code> for the specified command
		 * @param cmd name of the command
		 */
		public InvalidCommandException(String cmd) {
			super("Invalid command '" + cmd + "'");
		}
	}

	@SuppressWarnings("serial")
	public static class InvalidUsageException extends Exception {
		public InvalidUsageException() {
			super();
		}
		public InvalidUsageException(String message) {
			super(message);
		}
	}

	private static void printVersion() {
		System.out.println(Version.get());
	}

}