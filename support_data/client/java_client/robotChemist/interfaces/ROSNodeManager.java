package robotChemist.interfaces;

import java.net.InetAddress;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.UnknownHostException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.ros.address.BindAddress;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.time.NtpTimeProvider;
import org.ros.time.TimeProvider;
import org.ros.time.WallTimeProvider;

import com.kuka.task.ITaskLogger;

import robotChemist.nodes.LBRBaseNode;
import robotChemist.utility.AddressGenerator;

/***
 * This class manages ROS nodes and allows to use them inside Sunrise.OS. It is responsible 
 * for starting the nodes, configuring their settings and terminating them.
 * @author stoic-roboticist
 *
 */
public class ROSNodeManager {
	
	private ITaskLogger logger;
	
	// ROS nodes for communication
	private LBRBaseNode lbrNode = null;
	private ScheduledExecutorService ntpExecutorService = null;
	
	// ROS configurations
	private NodeConfiguration lbrNodeConfiguration = null;
	private TimeProvider timeProvider = null;
	private AddressGenerator addressGenerator = new AddressGenerator();
	private NodeMainExecutor nodeMainExecutor = null;
	private String masterIP;
	private String masterPort;
	private String robotIP;
	private String masterUri;
	private boolean useNtp = true;
	
	/***
	 * Constructs the ROS node manager with the given ROS master parameters. 
	 * @param robotName - the robot name used in status messages.
	 * @param masterIP - the ROS master IP address.
	 * @param masterPort - the ROS master port number.
	 * @param robotIP - the robot IP address, which is equivalent to its ROS_IP.
	 * @param useNtp - if true NTP server is used to sync timing. Otherwise, walltime is used. To avoid timing issues use NTP.
	 * @param logger - RoboticsAPI logging interface.
	 */
	public ROSNodeManager(String masterIP, String masterPort, String robotIP, boolean useNtp, ITaskLogger logger)
	{
		this.masterIP = masterIP;
		this.masterPort = masterPort;
		this.robotIP = robotIP;
		this.useNtp = useNtp;
		this.logger = logger;
	}
		
	/***
	 * Start ROS node by configuring it and then passing it down to the internal node executor.
	 * @throws UnknownHostException if ROS master settings are incorrect.
	 * @throws RuntimeException if an error occurs when starting the node executor.
	 */
	public void startNode(LBRBaseNode node) throws UnknownHostException, RuntimeException
	{
		masterUri = "http://" + masterIP + ":" + masterPort;
		configureTimeProvider();
		lbrNode = node;
		
		// ROS nodes initialisation
		try
		{
			lbrNodeConfiguration = configureNode(lbrNode.getDefaultNodeName().toString(), addressGenerator.getNewAddress(), 
					addressGenerator.getNewAddress());
			
		}
		catch (Exception e)
		{
			logger.error("Error when initializing ROS nodes");
			logger.error(e.getMessage());
			throw new RuntimeException("Could not initialize ROS nodes successfully.");
		}
		
		try
		{
			nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
			nodeMainExecutor.execute(lbrNode, lbrNodeConfiguration);
			logger.info("ROS node executor initialized");
		}
		catch (Exception e)
		{
			logger.error("Error when starting ROS node executor");
			logger.error(e.getMessage());
			throw new RuntimeException("Could not initialize ROS nodes successfully.");
		}
		
		//wait for ROS master
		logger.warn("waiting for ROS master");
		logger.warn("Application will terminate in 2 min if no master connects");
		long startTime = System.currentTimeMillis();
		while (!lbrNode.isConnectedToMaster())
		{
			if (System.currentTimeMillis() - startTime > 10000) //120000
			{
				logger.error("couldn't connect to master, exiting!!!");
				throw new RuntimeException(String.format("ROS Master: %s:%s is not available", masterIP, masterPort));
			}
		}
		logger.info("all nodes connected to the ROS master");
	}
	
	/***
	 * Get the managed node.
	 * @return managed ROS node
	 */
	public LBRBaseNode getNode()
	{
		return lbrNode;
	}
	
	private NodeConfiguration configureNode(String nodeName, int tcpPort, int xmlPort) throws URISyntaxException
	{
		NodeConfiguration nodeConfig = NodeConfiguration.newPublic(robotIP);
		nodeConfig.setTimeProvider(timeProvider);
		nodeConfig.setNodeName(nodeName);
		nodeConfig.setMasterUri(new URI(masterUri));
		nodeConfig.setTcpRosBindAddress(BindAddress.newPublic(tcpPort));
		nodeConfig.setXmlRpcBindAddress(BindAddress.newPublic(xmlPort));
		return nodeConfig;
	}
	
	private void configureTimeProvider() throws UnknownHostException
	{
		if (useNtp)
		{
			ntpExecutorService = Executors.newScheduledThreadPool(1);
			timeProvider = new NtpTimeProvider(InetAddress.getByName(masterIP), ntpExecutorService);
			((NtpTimeProvider) timeProvider).startPeriodicUpdates(100, TimeUnit.MILLISECONDS);
			logger.info("NTP server is used as a time provider");
		}
		else
		{
			timeProvider = new WallTimeProvider();
			logger.info("WallTime is used as a time provider");
		}
	}
	
	/***
	 * Shutdown the ROS node manager and terminates its nodes. Call this method 
	 * at the end of your application or as part of the dispose method to cleanly shutdown the manager.
	 */
	public void terminate()
	{
		logger.warn("Shutting down ROS node manager");
		shutDownExecutor(ntpExecutorService);
		// shutdown ROS node executor
		if (nodeMainExecutor != null) 
		{
			if (getNode() != null)
			{
				nodeMainExecutor.shutdownNodeMain(lbrNode);
			}
			logger.info("Shutting down ROS node executor");
			//nodeMainExecutor.shutdown();
			shutDownExecutor(nodeMainExecutor.getScheduledExecutorService());
		}
		logger.info("All shutdown cleanly");
	}
	
	private void shutDownExecutor(ScheduledExecutorService executor)
	{
		if (executor != null)
		{
			executor.shutdown();
			try
			{
				if (!executor.awaitTermination(5, TimeUnit.SECONDS))
				{
					executor.shutdownNow();
					logger.info("Executor service terminated forcebly");
				}
				else
				{
					logger.info("Executor service terminated cleanly");
				}
			}
			catch (InterruptedException e)
			{
				logger.error(e.getMessage());
				executor.shutdownNow();
			}
		}
	}

}
