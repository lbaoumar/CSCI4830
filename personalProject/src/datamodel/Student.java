package datamodel;
import javax.persistence.Column;
import javax.persistence.Entity;
import javax.persistence.GeneratedValue;
import javax.persistence.GenerationType;
import javax.persistence.Id;
import javax.persistence.Table;

/**
 * @since J2SE-1.8
 CREATE TABLE student (
  id INT NOT NULL AUTO_INCREMENT,    
  first name VARCHAR(30) NOT NULL,
  last name VARCHAR(30) NOT NULL,
  NUID VARCHAR(30) NOT NULL,   
  email VARCHAR(30) NOT NULL,,    
  PRIMARY KEY (id));
 */
@Entity
@Table(name = "Student")
public class Student {

   @Id  // primary key
   @GeneratedValue(strategy = GenerationType.IDENTITY)
   @Column(name = "id") // specify the column name. Without it, it will use method name
   private Integer id;

   @Column(name = "firstName")
   private String first_name;

   @Column(name = "lastName")
   private String last_name;
   
   @Column(name = "NUID")
   private String NUID;
   
   @Column(name = "EMAIL")
   private String email;

   public Student() {
   }

   public Student(Integer id, String firstName, String lastName, String nuid, String email) {
      this.id = id;
      this.first_name = firstName;
      this.last_name = lastName;
      this.NUID = nuid;
      this.email= email;
   }

   public Student(String firstName, String lastName, String nuid, String email) {
	  this.first_name = firstName;
      this.last_name = lastName;
      this.NUID = nuid;
      this.email= email;
   }

   public Integer getId() {
      return id;
   }

   public void setId(Integer id) {
      this.id = id;
   }

   public String getFirstName() {
      return first_name;
   }

   public void setFirstName(String firstName) {
	   this.first_name = firstName;
   }

   public String getLastName() {
	      return last_name;
	   }
   public void setLaststName(String lastName) {
	   this.last_name = lastName;
   }
   public void setNUID(String nuid) {
		   this.NUID = nuid;
	   }
   public String getNUID() {
	      return NUID;
	   }
   public void setemail(String email) {
	   this.email = email;
   }
public String getemail() {
      return email;
   }

   @Override
   public String toString() {
      return "Student: " + this.id + ", " + this.first_name + ", " + this.last_name + ", " + this.NUID + ", " + this.email;
   }
}