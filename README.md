# Hello-world
A sample project

import java.util.Scanner;

public class test {
	
	public static long convert(String origin) {
	  char[] characters = origin.toCharArray();
	  long result = 0;
	  
	  
	  for (int i = 0; i < characters.length; i++) {
		  result *= 10;
		  result += (characters[i] - 48);
	  }
	  
	  
	  return result;
	}
	

	public static String[] divide(String origin) {
	  String[] number = origin.split(" ");
	  String numbers = number[0];
	  String last = number[1];
	  long divisor = convert(number[1]);
	  
	  
	  String[] result = new String[numbers.length() + 1];
      for (int i = 0; i < numbers.length(); i++) {
    	  for (int j = 0; j < divisor; j++) {
    		  result[i] = numbers.substring(i, i + j);
    	  }
      }
      
      result[numbers.length()] = last; 
      
      return result;
	}
	
	
	public static long sum(String[] numbers) {
		long divisor = convert(numbers[numbers.length - 1]);
		long[] integers = new long[ (numbers.length - (int)divisor)];
		for (int i = 0; i < integers.length; i++) {
			for(int j = 0; j < (int)divisor; j++) {
				integers[i] *= 10;
				integers[i] += convert(numbers[i + j]);
			}
		}
		
		
		long result = 0;
		for (int i = 0; i < numbers.length - divisor; i++) {
			result += integers[i];
		}
		
		return result;
	}
	
	

	
	
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		
		
		//未成功!
    Scanner input = new Scanner(System.in);
    String originNumber = input.nextLine();
    System.out.println(sum(divide(originNumber)));
	}

}

