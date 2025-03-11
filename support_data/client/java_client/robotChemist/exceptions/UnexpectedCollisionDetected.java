package robotChemist.exceptions;

public class UnexpectedCollisionDetected extends Exception {
	
	private static final long serialVersionUID = 1L;
	public UnexpectedCollisionDetected(String error){
		super(error);
	}
}
