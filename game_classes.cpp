#include <iostream>
#include <string.h>

using namespace std;

class monster {
		int health, ammo;
		char *name;
	public:
		monster(int he=100, int am=10) :
			health (he), ammo (am), name ("default") {cout << "Default constructor... " << endl; }
        monster(int he, int am, char *n) {
            cout << "Constructor... "; health = he; ammo = am; name = new char[strlen(n)+1]; strcpy(name,n); cout << name << endl;}
		int get_health() {return health;}
		int get_ammo() {return ammo;}
		char* get_name()
        {
            return name;
        }

        char* change_name()
        {
            name = strcat(name,"NODE");
            return name;
        }

		void cure(int health, int ammo) {
			this->health += health;
			monster::ammo += ammo;
		}
		const monster& operator = (monster &M)
		{
		    cout << "Base = ... " << M.get_name() << endl;
		    if (&M == this) return *this;
		    if (name) delete [] name;
		    if (M.name)
            {
                name = new char[strlen(M.name)+1];
                strcpy(name,M.name);
            }
            else name = 0;
            health = M.health; ammo = M.ammo;
            return *this;
		}
		~monster()
		{
		    cout << name;
		    delete [] name;
		    cout << " ...destructed" << endl;
		}
};

class Headcrab : virtual public monster
{
        char* str;
    public:
        Headcrab() {cout << "Headcrab default constructor..." << endl; str = "defaultstr";}
        Headcrab(char* s, int he, int am, char* n) : monster(he,am,n) {cout << "Headcrab parameter constructor... " << endl; str = new char[strlen(s)+1]; strcpy(str,s); cout << str << endl;}
        ~Headcrab() {cout << "Destructor Headcrab... " << str << endl; delete [] str;}
        char* GetStr() {return str;}
        void Change(char* s) {strcpy(str,s);}
        const Headcrab& operator = (Headcrab &);

};

        const Headcrab& Headcrab::operator = (Headcrab &M)
        {
            cout << "Headcrab = ... " << endl;
		    if (&M == this) return *this;
		    if (str) delete [] str;
		    if (M.str)
            {
                str = new char[strlen(M.str)+1];
                strcpy(str,M.str);
            }
            else str = 0;
            monster::operator = (M);
            return *this;
        }

class Puppy : virtual public monster
{
    char* dogname;
    public:
        Puppy() {cout << "Puppy default constructor..." << endl; dogname = "defaultdog";}
        Puppy(char* s, int he, int am, char* n) : monster(he,am,n) {cout << "Puppy parameter constructor..." << endl; dogname = new char[strlen(s)+1]; strcpy(dogname,s); cout << dogname << endl;}
        ~Puppy() {cout << "Destructor Puppy... " << dogname << endl; delete [] dogname;}
        char* GetDog() {return dogname;}
        void Change(char* s) {strcpy(dogname,s);}
        const Puppy& operator = (Puppy &);
};

const Puppy& Puppy::operator = (Puppy &M)
        {
            cout << "Puppy = ... " << endl;
		    if (&M == this) return *this;
		    if (dogname) delete [] dogname;
		    if (M.dogname)
            {
                dogname = new char[strlen(M.dogname)+1];
                strcpy(dogname,M.dogname);
            }
            else dogname = 0;
            monster::operator = (M);
            return *this;
        }

class Baby : public Puppy, public Headcrab
{
        char *bname;
    public:
        Baby() {cout << "Baby default constructor..." << endl; bname = "defaultbabe";}
        Baby(char* s, char *ps, char *hs, int he, int am, char* n) : Headcrab(hs,he,am,n), Puppy(ps,he,am,n) {cout << "Babe parameter constructor..." << endl; bname = new char[strlen(s)+1]; strcpy(bname,s); cout << bname << endl;}
        ~Baby() {cout << "Destructor Baby... " << bname << endl; delete [] bname;}
        char* GetBabe() {return bname;}
        void Change(char* s) {strcpy(bname,s);}
};

struct Node
{
    monster d;
    Node *p;
};

void push(Node **top, monster &d)
{
    Node *pv = new Node;
    pv->d = d;
    pv->p = *top;
    *top = pv;
    cout << "In node " << (*top)->d.get_name() << endl;
    cout << (*top)->d.change_name() << endl;
}

void pop(Node **top)
{
    cout << "Deleting... " << (*top)->d.get_name() << endl;
    Node *pv = *top;
    *top = (*top)->p;
    delete pv;
}

void show(Node * const top)
{
    cout << (top)->d.get_name() << endl;
    cout  << (top)->d.get_health() << endl;
}
/*class Hero;
class Monster
{
    friend class Hero;
    friend bool operator > (Hero &, Monster &);
    private:
        int power, health;
    public:
        Monster(int pow, int he = 50) : power(pow), health(he) {};
        int GetHealth() {return health;}
};

class Hero
{
    friend bool operator > (Hero &, Monster &);
    private:
        int power;
    public:
        Hero(int pow) : power(pow) {};
        void Kill(Monster & monster) {monster.health = 0; cout << "Hit!" << endl;}
};

bool operator > (Hero & hero, Monster & monster)
{
    if (hero.power > monster.power) {cout << "You'll beat him!" << endl; return true;}
    else {cout << "Be careful, he's strong!" << endl; return false;}
}

enum Month {Jan = 1, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec};

Month operator ++(Month &m)
{
    m = (m==Dec) ? Jan : Month(m+1);
    return m;
    } */


int main()
{
    Baby mybabe("baby111", "puppy111", "chicken111", 666, 66, "MONSTER");
    cout << mybabe.GetBabe() << " " << mybabe.GetDog() << " " << mybabe.GetStr() << " " << mybabe.get_ammo() << endl;
    return 0;

    /*int menu, h, a;
    char n[100];
    Node *top = NULL;
    do
    {
        cin >> menu;
        switch (menu){
        case 1:
            {
                cout << "health?" << endl;
                cin >> h;
                cout << "ammo?" << endl;
                cin >> a;
                cout << "name?" << endl;
                cin >> n;
                monster *M = new monster(h,a,n);
                push(&top, *M);
                delete M;
                break;
            }
        case 2:
            {
                if(top) pop(&top);
                break;
            }
        case 3:
            {
                if(top) show(top);
                break;
            }
            default: cout << "retry" << endl;
        }
    } while (menu);*/

    /*monster headcrab;
    cout << headcrab.get_health() << "\t";
    headcrab.cure(15, 10);
    cout << headcrab.get_health() << endl;*/

    /*Headcrab chicken("ololo", 666, 47, "chick");
    cout << chicken.monster::get_name() << endl;
    cout << chicken.GetStr() << endl;

    Headcrab pock("ahaha", 666, 47, "pock");
    cout << pock.monster::get_name() << endl;
    cout << pock.GetStr() << endl;

    pock = chicken;
    cout << pock.get_name() << endl;
    cout << pock.GetStr() << endl;

    chicken.Change("br-a");
    cout << pock.get_name() << endl;
    cout << pock.GetStr() << endl;*/

    /*monster *A = new monster(50, 2, "head");
    cout << A->get_name() << "\n";
    delete A;
    monster *B = new monster[10];
    delete [] B;
    cin.get();*/

    /*Hero iamhero (100);
    Monster headcrab (10);
    cout << "Headcrab's health is " << headcrab.GetHealth() << endl;
    if (iamhero > headcrab) iamhero.Kill(headcrab);
    cout << "Headcrab's health is " << headcrab.GetHealth() << endl;*/

    /*Month m = May;
	cout << ++m; */
}

/*class Monster {
int health, ammo; char *name;
public:
explicit Monster (int he, int am, char *n) {
health = he; ammo = am;
name = new char[strlen(n) + 1]; }
};

int main() {
Monster m = {10, 20, "def"};
}*/

