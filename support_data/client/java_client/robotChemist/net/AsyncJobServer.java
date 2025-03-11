package robotChemist.net;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.nio.channels.SocketChannel;
import java.util.Iterator;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;


import com.kuka.task.ITaskLogger;

/***
 * This class allows asynchronous communication with an {@link AsyncJobClient} using sockets such that jobs messages 
 * can be exchanged between the arm and the base easily and with no blocking. Because of the way KMRiiwa is setup
 * such that LBR programs can be launched from the KMR Nav controller, this server is expected to be running on the KMR
 * Nav controller before launching any programs on the LBR controller that uses AsyncJobClient to communicate.
 * @author stoic-roboticist
 *
 */
public class AsyncJobServer {
	
	private ServerSocketChannel server;
	private SocketChannel client;
	private Selector selector;
	private ByteBuffer outBuffer;
	private ByteBuffer inBuffer;
	
	private LinkedBlockingQueue<JobMsg> inBoundMsgQueue;
	private JobMsg outMessage;
	
	private ExecutorService thread;
	private boolean running = false;
	
	private JobEncoder encoder;
	private JobDecoder decoder;
	
	private ITaskLogger logger;
	
	/***
	 * Constructs the AsyncJobServer object using the AsyncJobServer TCP/IP information.
	 * @param localAddress - the TCP/IP address to be used by the server.
	 * @param port - the TCP/IP port to be used by the server.
	 * @param logger - RoboticsAPI logging interface.
	 * @throws IOException if any of the underlying {@link ServerSocketChannel} methods fail.
	 */
	public AsyncJobServer(String localAddress, int port, ITaskLogger logger) throws IOException
	{
		this.selector = Selector.open();
		this.server = ServerSocketChannel.open();
		this.server.socket().bind(new InetSocketAddress(localAddress, port));
		this.server.configureBlocking(false);
		this.server.register(this.selector, SelectionKey.OP_ACCEPT);
		this.thread = Executors.newSingleThreadExecutor();
		this.inBuffer = ByteBuffer.allocate(256);
		this.outBuffer = ByteBuffer.allocate(256);
		this.inBoundMsgQueue = new LinkedBlockingQueue<JobMsg>();
		this.logger = logger;
		this.encoder = new JobEncoder();
		this.decoder = new JobDecoder();
		
	}
	
	/***
	 * Start the server to begin communicating with any AsyncJobClient that try to connect
	 * such that messages can be exchanged.
	 * This method launches a separate thread to handle the connection.
	 */
	public void start()
	{
		running = true;
		thread.execute(new Runnable() 
		{

			@Override
			public void run() {
				while (running)
				{
					try
					{
						if (selector.select() <= 0)
						{
							continue;
						}
			            Set<SelectionKey> selectedKeys = selector.selectedKeys();
			            Iterator<SelectionKey> iter = selectedKeys.iterator();
			            while (iter.hasNext())
			            {
			            	SelectionKey key = iter.next();
			            	if (key.isAcceptable())
			            	{
			            		registerClient();
			            	}			            	
			            	if (key.isReadable())
			            	{
			            		SocketChannel sc = (SocketChannel) key.channel();
			            		if (sc == client)
			            		{
			            			readMessageFromBuffer();
			            		}
			            	}
			            	iter.remove();
			            }
					}
					catch (IOException e)
					{
						logger.error(e.getMessage());
						running = false;
					}
					catch (InterruptedException e)
					{
						logger.error(e.getMessage());
						running = false;
					}
				}
				
			}
			
		});
	}
	
	/***
	 * Terminates the server and thus the established connection,and also stops the connection thread.
	 * This method has to be called when terminating any application using the AsyncJobServer class.
	 * @throws InterruptedException
	 * @throws IOException
	 */
	public void stop() throws InterruptedException, IOException
	{
		running = false;
		//thread.shutdown();
		if (!thread.awaitTermination(5, TimeUnit.SECONDS))
		{
			thread.shutdownNow();
		}
		if (client != null)
		{
			client.close();
		}
		selector.close();
		server.close();
	}
	
	private void registerClient() throws IOException
	{
		if (client == null)
		{
			client = server.accept();
	        client.configureBlocking(false);
	        client.register(selector, SelectionKey.OP_READ);
    		logger.info("client registeration complete");
		}
	}
	
	private void readMessageFromBuffer() throws IOException, InterruptedException
	{
		inBuffer.clear();
		int inLength = client.read(inBuffer);
		inBuffer.flip();
		if (inLength > 0)
		{
			JobMsg inMessage = decoder.decode(inBuffer);
			inBoundMsgQueue.put(inMessage);
		}
	}
	
	private void sendMessageFromBuffer() throws IOException
	{
		encoder.encode(outMessage, outBuffer);
		client.write(outBuffer);	
		outMessage = null;
	}
	
	/***
	 * Sends the given {@link JobMsg} to the client asynchronously.
	 * @param job - the message to be sent.
	 * @throws IOException
	 */
	public synchronized void sendMessage(JobMsg msg) throws IOException
	{
		outMessage = msg;
		sendMessageFromBuffer();
	}
	
	/***
	 * Gets the messages queue received from the client.
	 * @return  the received message queue LinkedBlockingQueue<JobMsg>.
	 */
	public LinkedBlockingQueue<JobMsg> getMessageQueue()
	{
		return inBoundMsgQueue;
	}

}
