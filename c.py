import time
import logging
from collections import deque

try:
    import lidar
except ImportError:
    try:
        import os, sys

        sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build'))
        import lidar
    except ImportError:
        print("Failed to import the lidar module. Please ensure the build path is imported correctly.")
        sys.exit(1)

from pylib.args import load_config, save_config
from pylib.args import get_client_parser, client_gui_args
from pylib.work import workflow, send_results_to_reporting_server, send_results_to_visualization_server

logger = logging.getLogger()


def pre_setup(args):
    """ Set up the configuration for a trail."""

    ## setup logging
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)

    log_file = os.path.join(args.LOGS_FOLDER, args.stamp, 'app.log')
    os.makedirs(os.path.dirname(log_file), exist_ok=True)
    formatter = logging.Formatter('[%(asctime)s] - (%(levelname)s): %(message)s')

    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(formatter)

    logfile_handler = logging.FileHandler(log_file)
    logfile_handler.setLevel(logging.DEBUG)
    logfile_handler.setFormatter(formatter)

    root_logger.addHandler(console_handler)
    root_logger.addHandler(logfile_handler)

    ## setup pcd folder
    os.makedirs(os.path.join(args.PCDS_FOLDER, args.stamp), exist_ok=True)

    logger.info("Configuration:")
    for k, v in vars(args).items():
        logger.info(f"  {k}: {v}")
    logger.info(f"Setup complete. Logs will be saved to {log_file}.")


def run_logic(args):
    """ Main process for Lidar detection.

    Args:
        args (argparse.Namespace): Parsed command line arguments.
    """
    ## initialize Lidar manager
    manager = lidar.LidarManager()
    logger.info("Starting Lidar...")
    if args.connect_type == 0:
        manager.initLidarWithUDP(
            args.lidar_ip, args.lidar_port,
            args.local_ip, args.local_port
        )
    elif args.connect_type == 1:
        manager.initLidarWithSerial()
    else:
        logger.error("Unsupported connection type. Use 0 for UDP and 1 for Serial.")
        return
    time.sleep(1)  ## wait for the Lidar to initialize
    manager.startLidar()
    time.sleep(args.START_LIDAR_WAIT_TIME)  ## wait for the Lidar to start

    ## Main loop to process point cloud data
    logger.info("Entering main loop...\n\n\n")
    try:
        assert args.HISTORY_WINDOW_SIZE <= 6, "HISTORY_WINDOW_SIZE should be at most 6."
        history = deque(maxlen=args.HISTORY_WINDOW_SIZE - 1)
        current_round = 1
        while True:
            start_time = time.time()
            logger.info('-' * 25 + ' Beigin Processing ' + '-' * 25)

            ## get results from workflow
            results = workflow(args, manager, history)
            if results is None:
                time.sleep(1)
                continue
            history.append({k: results[k] for k in ['volume', 'area', 'max_height', 'mean_height', 'lowest_z']})

            ## report results to the server if needed
            if (current_round >= args.start_upload_round) and (args.upload_data or args.upload_file):
                send_results_to_reporting_server(
                    args,
                    results,
                )

            ## visualize results if needed
            if args.visualize:
                send_results_to_visualization_server(
                    args,
                    results,
                )

            ## print results to console
            results_str = "Overall: V = {:.6f} m³, A = {:.6f} m², Max-H = {:.6f} m, Mean-H = {:.6f} m, Quadrant = {}.".format(
                results['volume'],
                results['area'],
                results['max_height'],
                results['mean_height'],
                results['quadrant'],
            )
            logger.info(results_str)
            logger.info('-' * 25 + '------- End -------' + '-' * 25 + '\n')

            ## sleep for the specified frequency
            computation_time = int(time.time() - start_time)
            waiting_time = args.report_interval - computation_time if args.report_interval > computation_time else 0
            logger.info(
                f"Processing time: {computation_time} seconds. Waiting for {waiting_time} seconds before the next cycle.")
            if args.enable_start_stop and waiting_time > 3 * args.START_LIDAR_WAIT_TIME:
                ## enable start/stop functionality only if the report interval is larger than 3 times of the start wait time
                logger.info("Stopping Lidar for the next cycle...")
                manager.stopLidar()
                time.sleep(waiting_time - args.START_LIDAR_WAIT_TIME)
                logger.info("Restarting Lidar...")
                manager.startLidar()
                time.sleep(args.START_LIDAR_WAIT_TIME)
            else:
                time.sleep(waiting_time)
            current_round += 1
            logger.info('Waiting is done. Continuing to the next cycle...\n\n\n')
    except KeyboardInterrupt:
        logger.info("Process interrupted by user.")
    except Exception as e:
        logger.error(f"An error occurred: {e}")
    finally:
        logger.info("Stopping Lidar...")
        manager.stopLidar()
        time.sleep(1)


def main():
    parser = get_client_parser()

    ## get pre-args for choosing GUI or CLI mode
    pre_args, _ = parser.parse_known_args()

    if pre_args.cli:
        args = parser.parse_args()
    else:
        args = client_gui_args(pre_args)

    if args.save:
        ## Save the configuration to the default config path
        saved_args = {**vars(args)}  # new dictionary to avoid modifying the original args
        saved_args.pop('cli', None)  # remove 'cli'
        save_config(args.DEFAULT_CONFIG_FILE, saved_args)

    ## Run the main logic of the application
    pre_setup(args)
    run_logic(args)


if __name__ == "__main__":
    main()